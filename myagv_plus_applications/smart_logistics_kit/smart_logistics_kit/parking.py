#!/usr/bin/env python3

import math
import re
import threading
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data

import cv2
import numpy as np
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation

import tf2_ros

from myagv_plus_msgs.action import Parking


GREEN = '\033[32m'
RED = '\033[31m'
RESET = '\033[0m'


class AutomaticParkingVision(Node):

    def __init__(self):
        super().__init__('automatic_parking_vision')

        # ===== SUB =====
        self.sub_odom_robot = self.create_subscription(
            Odometry,
            '/odom',
            self.get_robot_odom,
            QoSProfile(depth=10)
        )

        self.sub_camera_image = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_cb,
            qos_profile_sensor_data
        )
        self.parking_reset_sub = self.create_subscription(Bool, 'parking/reset', self.reset_cb, 10)

        # ===== PUB =====
        self.pub_cmd_vel = self.create_publisher(
            TwistStamped,
            '/cmd_vel',
            QoSProfile(depth=10)
        )

        detect_qos = QoSProfile(depth=1)
        detect_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.detect_enable_pub = self.create_publisher(Bool, 'parking/detect_enable', detect_qos)
        self._detect_enable_last = None

        # ===== ACTION SERVER =====
        self._action_server = ActionServer(
            self,
            Parking,
            'parking_action',
            execute_callback=self.execute_callback
        )

        self.get_logger().info('Parking Action Server Ready')

        # ===== STATE MACHINE =====
        self.ParkingSequence = Enum(
            'ParkingSequence',
            'waiting ocr_check marker_warmup search align align_y align_yaw measure parking done'
        )

        # ---------------- VARIABLES ----------------
        self.target_marker_id = None
        self.marker_frame = None
        self.selected_marker_frame_logged = False
        self.last_state_log = None
        self.last_state_log_time = 0.0

        self.current_state = self.ParkingSequence.waiting
        
        self.robot_2d_pose_x = 0.0
        self.robot_2d_pose_y = 0.0
        self.robot_2d_theta = 0.0

        self.marker_2d_pose_x = 0.0
        self.marker_2d_pose_y = 0.0
        self.marker_2d_theta = 0.0
        self.cached_marker_pose = None

        self.parking_target_marker_distance = 0.195
        self.parking_distance_tolerance = 0.01
        self.min_angular_z = 0.1
        self.align_y_tolerance = 0.005  # Y 向对齐容差，单位米；Y alignment tolerance, in meters.
        self.align_y_target_offset = 0.00  # Y 向额外偏移距离，单位米；Extra Y target offset, in meters. 0 means align marker_y to 0.
        self.align_y_target_pose_y = None
        self.align_y_kp = 1.5  # Y 向横移比例增益；Y lateral alignment proportional gain.
        self.align_y_direction = 1.0  # Y 向横移方向修正；Y lateral direction sign correction.
        self.align_yaw_snap_window = math.radians(10.0)  # 判定可吸附到 0/π 的角度窗口；Snap window to 0/pi.
        self.align_yaw_tolerance = math.radians(1.5)  # 吸附到位角度容差，单位弧度；Yaw snap tolerance, in rad.
        self.align_yaw_kp = 0.6  # yaw 吸附旋转比例增益；Yaw snap proportional gain.
        self.align_yaw_min_angular_z = 0.05  # yaw 吸附最小角速度，单位 rad/s；Min yaw angular speed.
        self.align_yaw_max_angular_z = 0.08  # yaw 吸附最大角速度，单位 rad/s；Max yaw angular speed.
        self.align_yaw_target = None  # 本次吸附目标 map yaw（0 或 π）；Snap target map yaw.
        self.min_linear_y = 0.04  # Y 向最小横移速度，单位 m/s；Minimum lateral Y speed, in m/s.
        self.max_linear_y = 0.08  # Y 向最大横移速度，单位 m/s；Maximum lateral Y speed, in m/s.
        self.parking_start_pose_x = 0.0
        self.parking_start_pose_y = 0.0
        self.parking_target_distance = 0.0
        self.marker_measure_count = 5
        self.marker_measure_samples = []
        self.marker_tf_timeout = 0.5
        self.cached_marker_timeout = 2.0
        self.marker_tf_start_time = None
        self.marker_stall_timeout = 0.2
        self.last_align_angle = None
        self.last_align_update_time = time.monotonic()

        self.previous_robot_2d_theta = 0.0
        self.total_robot_2d_theta = 0.0

        self.is_sequence_finished = False
        self.is_parking_retried = False

        self.is_odom_received = False
        self.is_marker_pose_received = False


        # ===== TF =====
        self.marker_frame = None
        self.is_marker_pose_received = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ===== ODOM =====
        self.robot_2d_theta = 0.0

        # ===== TIMER =====
        self.timer = self.create_timer(0.1, self.get_marker_odom)
        self.timer = self.create_timer(0.1, self._run)

        # ===== ACTION CONTROL =====
        self._goal_handle = None
        self._goal_active = False
        self._goal_token = 0
        self._done = False
        self._ready_logged = False
        self._startup_wait_timer = self.create_timer(5.0, self._log_startup_wait)

        self.bridge = CvBridge()
        self.latest_image_msg = None
        self.expect_region = ''
        self.ocr_window = 'OCR Region Check'
        self.ocr_hold_seconds = 1.0
        self.last_ocr_log_time = 0.0
        self.ocr_active = False
        self.ocr_matched = False
        self.ocr_latest_annotated = None
        self.ocr_matched_frame = None
        self.ocr_done = False
        self.ocr_recognize_thread = None
        self.ocr_display_thread = None
        self.marker_settle_seconds = 1.0
        self.marker_first_seen_time = None
        self.marker_settle_last = None
        self.marker_stable_count = 0
        self.marker_stable_needed = 3
        self.marker_stable_xy_tol = 0.005
        self.marker_stable_theta_tol = 0.02
        self.marker_warmup_seconds = 1.5
        self.marker_warmup_stable_needed = 3
        self.marker_warmup_start_time = None
        self.marker_warmup_samples = []
        self.ocr = None
        self.ocr_error = None
        try:
            from smart_logistics_kit.OCRVideoCapture import OCRVideoCapture
            self.ocr = OCRVideoCapture()
            self.get_logger().info(f'{GREEN}OCR model loaded.{RESET}')
        except Exception as error:
            self.ocr_error = f'{type(error).__name__}: {error}'
            self.get_logger().error(
                f'{RED}OCR import/load failed during parking startup: '
                f'{self.ocr_error}. Parking action server is still ready, '
                f'but destination region OCR check cannot run.{RESET}')

    def image_cb(self, msg):
        self.latest_image_msg = msg

    def _log_startup_wait(self):
        if not self.is_odom_received:
            self.get_logger().warn('Parking startup waiting: action server ready, waiting for /odom.')

    # ================= ACTION =================
    def reset_cb(self, msg):
        if not msg.data:
            return
        self.get_logger().warn('Parking reset requested. Stop current parking goal and return to waiting.')
        self._goal_token += 1
        self.finish_parking_goal()

    def execute_callback(self, goal_handle):
        self.get_logger().info('Parking Goal Received')

        if self._goal_active:
            self.get_logger().warn('New parking goal received while previous goal is active. Reset previous parking state.')
            self.fn_stop()
        self._goal_token += 1
        goal_token = self._goal_token
        self._goal_active = True
        self._goal_handle = goal_handle
        self.reset_parking_state(goal_handle.request.marker_id, goal_handle.request.expect_region)
        if self.target_marker_id < 0:
            self.get_logger().info('Parking will use any visible ArUco marker.')

        if self.expect_region:
            self.current_state = self.ParkingSequence.ocr_check
        else:
            self.current_state = self.ParkingSequence.marker_warmup

        while (
                rclpy.ok()
                and self._goal_active
                and self._goal_token == goal_token
                and not self._done):
            if goal_handle.is_cancel_requested:
                self.fn_stop()
                self._done = True
                break
            time.sleep(0.1)

        result = Parking.Result()
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result.success = False
            result.message = "Parking canceled"
            self.get_logger().warn('Parking Action Canceled')
        elif self._goal_token != goal_token:
            goal_handle.abort()
            result.success = False
            result.message = "Parking preempted by a newer goal"
            self.get_logger().warn('Parking Action Preempted')
        elif self._done:
            goal_handle.succeed()
            result.success = True
            result.message = "Parking completed"
            self.get_logger().info('Action finished successfully')
        else:
            goal_handle.abort()
            result.success = False
            result.message = "Parking interrupted"
            self.get_logger().warn('Parking Action Interrupted')

        if self._goal_token == goal_token:
            self.finish_parking_goal()

        return result

    def finish_parking_goal(self):
        self.fn_stop()
        self._goal_active = False
        self._goal_handle = None
        self._done = False
        self.current_state = self.ParkingSequence.waiting
        self.publish_detect_enable()

    def reset_parking_state(self, marker_id, expect_region=''):
        self._done = False
        self.target_marker_id = marker_id
        self.expect_region = expect_region
        self.ocr_active = False
        self.ocr_matched = False
        self.ocr_latest_annotated = None
        self.ocr_matched_frame = None
        self.ocr_done = False
        self.marker_first_seen_time = None
        self.marker_settle_last = None
        self.marker_stable_count = 0
        self.marker_warmup_start_time = None
        self.marker_warmup_samples = []
        self.marker_frame = None if marker_id < 0 else f'ar_marker_{marker_id}'
        self.selected_marker_frame_logged = False
        self.marker_detected_once = False
        self.marker_2d_pose_x = 0.0
        self.marker_2d_pose_y = 0.0
        self.marker_2d_theta = 0.0
        self.cached_marker_pose = None
        self.is_marker_pose_received = False
        self.is_sequence_finished = False
        self.is_parking_retried = False
        self.last_state_log = None
        self.last_state_log_time = 0.0
        self.parking_start_pose_x = 0.0
        self.parking_start_pose_y = 0.0
        self.parking_target_distance = 0.0
        self.marker_measure_samples = []
        self.marker_tf_start_time = self.get_clock().now()
        self.last_align_angle = None
        self.last_align_update_time = time.monotonic()
        self.align_y_target_pose_y = None

    def marker_phase(self):
        if not self._goal_active:
            return False
        if self.current_state == self.ParkingSequence.ocr_check:
            return self.ocr_matched
        return self.current_state in (
            self.ParkingSequence.marker_warmup, self.ParkingSequence.search,
            self.ParkingSequence.align, self.ParkingSequence.align_y,
            self.ParkingSequence.measure)

    def publish_detect_enable(self):
        enable = self.marker_phase()
        if enable != self._detect_enable_last:
            self._detect_enable_last = enable
            self.detect_enable_pub.publish(Bool(data=enable))

    # ================= TIMER =================
    def _run(self):
        self.publish_detect_enable()

        if not self._goal_active:
            if self.current_state != self.ParkingSequence.waiting:
                self.fn_stop()
                self.current_state = self.ParkingSequence.waiting
                self.last_state_log = None
            return

        if self.current_state == self.ParkingSequence.waiting:
            return

        state = (self.current_state, self.is_marker_pose_received)
        now = time.monotonic()
        if state != self.last_state_log or now - self.last_state_log_time >= 2.0:
            self.last_state_log = state
            self.last_state_log_time = now
            self.get_logger().info(
                f'[STATE] {self.current_state}, marker={self.is_marker_pose_received}'
            )
        

        # self.get_marker_pose()

        if self.current_state == self.ParkingSequence.search:
            self.is_sequence_finished = self.seq_find_goal()
            if self.is_sequence_finished:
                self.get_logger().info('Finished find goal sequence')
                self.is_sequence_finished = False
                self.last_align_angle = None
                self.last_align_update_time = time.monotonic()
                self.current_state = self.ParkingSequence.align

        elif self.current_state == self.ParkingSequence.align:
            self.is_sequence_finished = self.seq_align_direction()
            if self.is_sequence_finished:
                self.get_logger().info('align_direction done')
                self.is_sequence_finished = False
                self.current_state = self.ParkingSequence.align_y

        elif self.current_state == self.ParkingSequence.ocr_check:
            self.fn_stop()
            if self.seq_ocr_check():
                self.get_logger().info('ocr_check done')
                self.marker_warmup_start_time = None
                self.marker_warmup_samples = []
                if self.is_marker_pose_received:
                    self.marker_warmup_samples.append((
                        self.marker_2d_pose_x,
                        self.marker_2d_pose_y,
                        self.marker_2d_theta,
                        time.monotonic(),
                    ))
                self.current_state = self.ParkingSequence.marker_warmup

        elif self.current_state == self.ParkingSequence.marker_warmup:
            if self.seq_marker_warmup():
                if self.marker_warmup_samples:
                    self.get_logger().info(
                        f'Marker warmup got {len(self.marker_warmup_samples)} frame(s); '
                        f'skip search and align with {self.marker_frame}.')
                    self.last_align_angle = None
                    self.last_align_update_time = time.monotonic()
                    self.current_state = self.ParkingSequence.align
                else:
                    self.get_logger().warn('Marker warmup got no fresh marker frame; enter search.')
                    self.current_state = self.ParkingSequence.search

        elif self.current_state == self.ParkingSequence.align_y:
            self.is_sequence_finished = self.seq_align_y()
            if self.is_sequence_finished:
                self.get_logger().info('align_y done')
                self.is_sequence_finished = False
                self.align_yaw_target = None
                self.current_state = self.ParkingSequence.align_yaw

        elif self.current_state == self.ParkingSequence.align_yaw:
            self.is_sequence_finished = self.seq_align_yaw()
            if self.is_sequence_finished:
                self.get_logger().info('align_yaw done')
                self.is_sequence_finished = False
                self.marker_measure_samples = []
                self.current_state = self.ParkingSequence.measure

        elif self.current_state == self.ParkingSequence.measure:
            self.fn_stop()
            if self.is_marker_pose_received:
                self.marker_measure_samples.append(abs(self.marker_2d_pose_x))

            if len(self.marker_measure_samples) >= self.marker_measure_count:
                marker_distance = sum(self.marker_measure_samples) / len(self.marker_measure_samples)
                self.parking_start_pose_x = self.robot_2d_pose_x
                self.parking_start_pose_y = self.robot_2d_pose_y
                self.parking_target_distance = max(
                    marker_distance - self.parking_target_marker_distance,
                    0.0)
                self.get_logger().info(
                    f'Parking marker average={marker_distance:.3f} m, '
                    f'odom target distance={self.parking_target_distance:.3f} m')
                self.current_state = self.ParkingSequence.parking

        elif self.current_state == self.ParkingSequence.parking:
            self.is_sequence_finished = self.seq_parking()
            if self.is_sequence_finished:
                self.get_logger().info('parking done')
                self.current_state = self.ParkingSequence.done
                self.fn_stop()

                self.target_marker_id = None
                self.marker_frame = None
                self.robot_2d_pose_x = .0
                self.robot_2d_pose_y = .0
                self.robot_2d_theta = .0
                self.marker_2d_pose_x = .0
                self.marker_2d_pose_y = .0
                self.marker_2d_theta = .0

                self.previous_robot_2d_theta = .0
                self.total_robot_2d_theta = .0

                self.is_sequence_finished = False
                self.is_parking_retried = False

                self.is_odom_received = False
                self.is_marker_pose_received = False

        elif self.current_state == self.ParkingSequence.done:
            self.get_logger().info('Parking Action Succeeded')
            self._done = True
            self.current_state = self.ParkingSequence.waiting

    # ================= TF =================
    def get_marker_odom(self):
        if not self.marker_phase():
            self.is_marker_pose_received = False
            return
        if self.marker_frame is None:
            if self.target_marker_id is not None and self.target_marker_id < 0:
                self.marker_frame = self.find_visible_marker_frame()
                if self.marker_frame is not None and not self.selected_marker_frame_logged:
                    self.selected_marker_frame_logged = True
                    self.get_logger().info(f'Parking selected marker: {self.marker_frame}')
            if self.marker_frame is None:
                self.is_marker_pose_received = False
                return

        allow_last_frame = self.current_state == self.ParkingSequence.search
        result = self.get_2D_marker_pose(allow_last_frame)
        if result is None:
            self.is_marker_pose_received = False
            if (
                self.target_marker_id is not None
                and self.target_marker_id < 0
                and self.get_cached_marker_pose() is None
            ):
                self.marker_frame = None
            return
        pos_x, pos_y, theta = result

        self.marker_2d_pose_x = pos_x
        self.marker_2d_pose_y = pos_y
        self.marker_2d_theta = theta - math.pi
        self.cached_marker_pose = (self.marker_2d_pose_x, self.marker_2d_pose_y, time.monotonic())

        self.is_marker_pose_received = True

    def get_2D_marker_pose(self, allow_last_frame=False):
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link', self.marker_frame, rclpy.time.Time().to_msg())
        except Exception:
            return
        timeout = self.cached_marker_timeout if allow_last_frame else self.marker_tf_timeout
        if not self.is_fresh_transform(trans, allow_last_frame, timeout):
            return

        quaternion = (
            trans.transform.rotation.x,
            trans.transform.rotation.y,
            trans.transform.rotation.z,
            trans.transform.rotation.w
        )

        rotation = Rotation.from_quat(quaternion)
        euler_angles = rotation.as_euler('xyz', degrees=False)
        theta = euler_angles[2]
        theta = theta + np.pi / 2.

        theta = theta % (2 * np.pi)

        pos_x = trans.transform.translation.x
        pos_y = trans.transform.translation.y

        return pos_x, pos_y, theta

    def is_fresh_transform(self, trans, allow_before_start=False, timeout=None):
        stamp = rclpy.time.Time.from_msg(trans.header.stamp)
        if not allow_before_start and self.marker_tf_start_time is not None:
            if (stamp - self.marker_tf_start_time).nanoseconds < 0:
                return False
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return age <= (self.marker_tf_timeout if timeout is None else timeout)

    def find_visible_marker_frame(self):
        try:
            frames = self.tf_buffer.all_frames_as_yaml()
        except Exception:
            return None

        marker_frames = sorted(
            set(re.findall(r'ar_marker_\d+', frames)),
            key=lambda frame: int(frame.rsplit('_', 1)[1]))
        for marker_frame in marker_frames:
            try:
                trans = self.tf_buffer.lookup_transform(
                    'base_link', marker_frame, rclpy.time.Time().to_msg())
            except Exception:
                continue
            if self.is_fresh_transform(trans, True, self.cached_marker_timeout):
                return marker_frame
        return None

    # ================= ODOM =================
    def get_robot_odom(self, robot_odom_msg):
        if not self.is_odom_received:
            self.is_odom_received = True
            if not self._ready_logged:
                self._ready_logged = True
                if self._startup_wait_timer is not None:
                    self.destroy_timer(self._startup_wait_timer)
                    self._startup_wait_timer = None
                self.get_logger().info(
                    f'{GREEN}Parking initialized: action server ready, odom received.{RESET}')

        pos_x, pos_y, theta = self.get_2D_robot_pose(robot_odom_msg)

        self.robot_2d_pose_x = pos_x
        self.robot_2d_pose_y = pos_y
        self.robot_2d_theta = theta

        if (self.robot_2d_theta - self.previous_robot_2d_theta) > 5.:
            d_theta = (self.robot_2d_theta - self.previous_robot_2d_theta) - 2 * math.pi
        elif (self.robot_2d_theta - self.previous_robot_2d_theta) < -5.:
            d_theta = (self.robot_2d_theta - self.previous_robot_2d_theta) + 2 * math.pi
        else:
            d_theta = (self.robot_2d_theta - self.previous_robot_2d_theta)

        self.total_robot_2d_theta = self.total_robot_2d_theta + d_theta
        self.previous_robot_2d_theta = self.robot_2d_theta

        self.robot_2d_theta = self.total_robot_2d_theta

    def get_2D_robot_pose(self, robot_odom_msg):
        quaternion = (
            robot_odom_msg.pose.pose.orientation.x,
            robot_odom_msg.pose.pose.orientation.y,
            robot_odom_msg.pose.pose.orientation.z,
            robot_odom_msg.pose.pose.orientation.w)

        r = Rotation.from_quat(quaternion)
        euler_angles = r.as_euler('xyz', degrees=False)
        theta = euler_angles[2]

        theta = theta % (2 * np.pi)

        pos_x = robot_odom_msg.pose.pose.position.x
        pos_y = robot_odom_msg.pose.pose.position.y

        return pos_x, pos_y, theta

    # ================= CONTROL =================

    def seq_find_goal(self):
        has_marker = self.is_marker_pose_received or self.get_cached_marker_pose() is not None
        if self.marker_first_seen_time is None:
            if has_marker:
                self.marker_first_seen_time = time.monotonic()
                self.marker_settle_last = (self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_theta)
                self.marker_stable_count = 0
                self.fn_stop()
            else:
                self.fn_turn(-0.15)
            return False

        self.fn_stop()
        pose = (self.marker_2d_pose_x, self.marker_2d_pose_y, self.marker_2d_theta)
        if self.marker_settle_last is not None:
            dx = abs(pose[0] - self.marker_settle_last[0])
            dy = abs(pose[1] - self.marker_settle_last[1])
            dth = abs(pose[2] - self.marker_settle_last[2])
            if dx < self.marker_stable_xy_tol and dy < self.marker_stable_xy_tol and dth < self.marker_stable_theta_tol:
                self.marker_stable_count += 1
            else:
                self.marker_stable_count = 0
        self.marker_settle_last = pose

        elapsed = time.monotonic() - self.marker_first_seen_time
        if self.marker_stable_count >= self.marker_stable_needed or elapsed >= self.marker_settle_seconds:
            self.marker_detected_once = True
            return True
        return False

    def seq_marker_warmup(self):
        self.fn_stop()
        if self.marker_warmup_start_time is None:
            self.marker_warmup_start_time = time.monotonic()

        if self.is_marker_pose_received:
            self.marker_warmup_samples.append((
                self.marker_2d_pose_x,
                self.marker_2d_pose_y,
                self.marker_2d_theta,
                time.monotonic(),
            ))

        return (
            self.has_stable_marker_warmup_sample()
            or time.monotonic() - self.marker_warmup_start_time >= self.marker_warmup_seconds
        )

    def has_stable_marker_warmup_sample(self):
        if len(self.marker_warmup_samples) < self.marker_warmup_stable_needed:
            return False

        recent = self.marker_warmup_samples[-self.marker_warmup_stable_needed:]
        first = recent[0]
        for sample in recent[1:]:
            dx = abs(sample[0] - first[0])
            dy = abs(sample[1] - first[1])
            dth = abs(math.atan2(math.sin(sample[2] - first[2]), math.cos(sample[2] - first[2])))
            if dx >= self.marker_stable_xy_tol or dy >= self.marker_stable_xy_tol or dth >= self.marker_stable_theta_tol:
                return False
        return True

    def seq_align_direction(self):
        marker_pose = self.get_align_marker_pose()
        if marker_pose is None:
            self.fn_turn(-0.20)
            return False

        marker_x, marker_y = marker_pose
        desired_angle_turn = -0.5 * math.atan2(marker_y, marker_x)
        print(f'desired_angle_turn: {desired_angle_turn}')
        self.fn_turn(desired_angle_turn)

        if abs(desired_angle_turn) < 0.01:
            self.fn_stop()
            return True
        elif self.last_align_angle is not None and abs(desired_angle_turn - self.last_align_angle) < 0.001:
            if time.monotonic() - self.last_align_update_time > self.marker_stall_timeout:
                self.fn_stop()
                self.get_logger().warn('Align angle stopped updating, finish align.')
                return True
        else:
            self.last_align_angle = desired_angle_turn
            self.last_align_update_time = time.monotonic()
        return False

    def get_align_marker_pose(self):
        if self.is_marker_pose_received:
            return self.marker_2d_pose_x, self.marker_2d_pose_y
        if not self.marker_detected_once or self.cached_marker_pose is None:
            return None
        return self.get_cached_marker_pose()

    def get_cached_marker_pose(self):
        if self.cached_marker_pose is None:
            return None
        marker_x, marker_y, stamp = self.cached_marker_pose
        if time.monotonic() - stamp > self.cached_marker_timeout:
            return None
        return marker_x, marker_y

    def throttled_ocr_log(self, message):
        now = time.monotonic()
        if now - self.last_ocr_log_time >= 1.0:
            self.last_ocr_log_time = now
            self.get_logger().error(f'{RED}{message}{RESET}')

    def frame_from_latest(self):
        msg = self.latest_image_msg
        if msg is None:
            return None
        try:
            return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            return None

    def ocr_recognize_loop(self):
        while self.ocr_active and not self.ocr_matched and rclpy.ok():
            frame = self.frame_from_latest()
            if frame is None:
                time.sleep(0.05)
                continue
            annotated, texts = self.ocr.recognize(frame)
            self.ocr_latest_annotated = annotated
            if self.expect_region in texts:
                self.ocr_matched_frame = annotated
                self.ocr_matched = True
                self.get_logger().info(f'{GREEN}分区匹配 ({self.expect_region}){RESET}')
            else:
                self.throttled_ocr_log(
                    f'该区域应放置 {self.expect_region} / This spot should hold '
                    f'{self.expect_region}. Waiting for the correct basket...')

    def ocr_display_loop(self):
        matched_since = None
        while self.ocr_active and rclpy.ok():
            if self.ocr_matched:
                if matched_since is None:
                    matched_since = time.monotonic()
                if self.ocr_matched_frame is not None:
                    cv2.imshow(self.ocr_window, self.ocr_matched_frame)
                cv2.waitKey(1)
                if time.monotonic() - matched_since >= self.ocr_hold_seconds:
                    break
            else:
                frame = self.ocr_latest_annotated
                if frame is None:
                    frame = self.frame_from_latest()
                if frame is not None:
                    cv2.imshow(self.ocr_window, frame)
                cv2.waitKey(1)
            time.sleep(0.03)
        try:
            cv2.destroyWindow(self.ocr_window)
            cv2.waitKey(1)
        except cv2.error:
            pass
        self.ocr_done = True

    def seq_ocr_check(self):
        if not self.expect_region:
            return True
        if self.ocr is None:
            detail = f' Startup error: {self.ocr_error}.' if self.ocr_error else ''
            self.throttled_ocr_log(f'OCR model unavailable; cannot verify region.{detail}')
            return False
        if not self.ocr_active:
            self.ocr_active = True
            self.ocr_matched = False
            self.ocr_matched_frame = None
            self.ocr_done = False
            self.ocr_recognize_thread = threading.Thread(target=self.ocr_recognize_loop, daemon=True)
            self.ocr_display_thread = threading.Thread(target=self.ocr_display_loop, daemon=True)
            self.ocr_recognize_thread.start()
            self.ocr_display_thread.start()
            return False
        if self.ocr_done:
            self.ocr_active = False
            return True
        return False

    def seq_align_y(self):
        if not self.is_marker_pose_received:
            self.fn_stop()
            return False

        if self.align_y_target_pose_y is None:
            extra_offset = abs(self.align_y_target_offset)
            if extra_offset > 0.0 and abs(self.marker_2d_pose_y) > self.align_y_tolerance:
                self.align_y_target_pose_y = -math.copysign(extra_offset, self.marker_2d_pose_y)
            else:
                self.align_y_target_pose_y = 0.0
            self.get_logger().info(
                f'Align y target: marker_y={self.marker_2d_pose_y:.3f} m, '
                f'target_y={self.align_y_target_pose_y:.3f} m.')

        error_y = self.marker_2d_pose_y - self.align_y_target_pose_y

        if abs(error_y) < self.align_y_tolerance:
            self.fn_stop()
            return True

        self.fn_shift_y(error_y)
        return False

    def get_map_yaw(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time().to_msg())
        except Exception:
            return None
        q = trans.transform.rotation
        yaw = Rotation.from_quat((q.x, q.y, q.z, q.w)).as_euler('xyz')[2]
        return math.atan2(math.sin(yaw), math.cos(yaw))

    def seq_align_yaw(self):
        yaw = self.get_map_yaw()
        if yaw is None:
            self.fn_stop()
            return False

        if self.align_yaw_target is None:
            to_zero = abs(math.atan2(math.sin(yaw), math.cos(yaw)))
            to_pi = abs(math.atan2(math.sin(yaw - math.pi), math.cos(yaw - math.pi)))
            nearest = 0.0 if to_zero <= to_pi else math.pi
            offset = min(to_zero, to_pi)
            if offset > self.align_yaw_snap_window:
                self.get_logger().warn(
                    f'Skip yaw snap: map yaw={math.degrees(yaw):.1f} deg not within '
                    f'{math.degrees(self.align_yaw_snap_window):.0f} deg of 0/180.')
                self.fn_stop()
                return True
            self.align_yaw_target = nearest
            self.get_logger().info(
                f'Yaw snap: map yaw={math.degrees(yaw):.1f} deg -> '
                f'{math.degrees(nearest):.0f} deg.')

        error = math.atan2(
            math.sin(self.align_yaw_target - yaw), math.cos(self.align_yaw_target - yaw))
        if abs(error) < self.align_yaw_tolerance:
            self.fn_stop()
            return True

        angular_z = self.align_yaw_kp * error
        magnitude = min(max(abs(angular_z), self.align_yaw_min_angular_z), self.align_yaw_max_angular_z)
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.angular.z = math.copysign(magnitude, error)
        self.pub_cmd_vel.publish(twist)
        return False

    def seq_parking(self):
        traveled_dist = self.calculate_distance_points(
            self.parking_start_pose_x, self.robot_2d_pose_x,
            self.parking_start_pose_y, self.robot_2d_pose_y)
        self.get_logger().info(f'{traveled_dist}')
        if traveled_dist + self.parking_distance_tolerance >= self.parking_target_distance:
            self.fn_stop()
            return True
        else:
            self.fn_track_marker(0.0)
            return False

    def calculate_distance_points(self, x1, x2, y1, y2):
        return math.sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

    # ================= MOTION =================
    def fn_stop(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = 0.0
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist)

    def fn_turn(self, theta):
        Kp = 1.0 #0.6
        angular_z = Kp * theta
        if abs(angular_z) > 0.0 and abs(angular_z) < self.min_angular_z:
            angular_z = self.min_angular_z if angular_z > 0.0 else -self.min_angular_z

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = 0.0
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = -angular_z
        self.pub_cmd_vel.publish(twist)

    def fn_shift_y(self, error_y):
        linear_y = self.align_y_direction * self.align_y_kp * error_y
        if abs(linear_y) > 0.0 and abs(linear_y) < self.min_linear_y:
            linear_y = self.min_linear_y if linear_y > 0.0 else -self.min_linear_y
        if abs(linear_y) > self.max_linear_y:
            linear_y = self.max_linear_y if linear_y > 0.0 else -self.max_linear_y

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = 0.0
        twist.twist.linear.y = linear_y
        twist.twist.linear.z = 0.0
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist)

    def fn_track_marker(self, theta):
        Kp = 0.8#0.6

        angular_z = Kp * theta
        if abs(angular_z) > 0.0 and abs(angular_z) < self.min_angular_z:
            angular_z = self.min_angular_z if angular_z > 0.0 else -self.min_angular_z

        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.twist.linear.x = 0.13#0.1
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = -angular_z
        self.pub_cmd_vel.publish(twist)


# ================= MAIN =================
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AutomaticParkingVision()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f'{RED}[parking] startup failed: {type(error).__name__}: {error}{RESET}')
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
