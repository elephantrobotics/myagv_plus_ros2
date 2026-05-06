#!/usr/bin/env python3

import math
import time
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

import numpy as np
from scipy.spatial.transform import Rotation

import tf2_ros

from myagv_plus_msgs.action import Parking


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

        # ===== PUB =====
        self.pub_cmd_vel = self.create_publisher(
            Twist,
            '/cmd_vel',
            QoSProfile(depth=10)
        )

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
            'waiting search align move parking done'
        )

        self.NearbySequence = Enum(
            'NearbySequence',
            'initial_turn go_straight turn_right parking'
        )

        # ---------------- VARIABLES ----------------
        self.target_marker_id = None
        self.marker_frame = None

        self.current_state = self.ParkingSequence.waiting
        self.current_nearby_sequence = self.NearbySequence.initial_turn
        
        self.robot_2d_pose_x = 0.0
        self.robot_2d_pose_y = 0.0
        self.robot_2d_theta = 0.0

        self.marker_2d_pose_x = 0.0
        self.marker_2d_pose_y = 0.0
        self.marker_2d_theta = 0.0

        self.parking_distance_to_marker = 0.285

        self.previous_robot_2d_theta = 0.0
        self.total_robot_2d_theta = 0.0

        self.is_triggered = False

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
        self._done = False

    # ================= ACTION =================
    def execute_callback(self, goal_handle):
        self.get_logger().info('Parking Goal Received')

        self._goal_handle = goal_handle
        self._done = False
        
        self.marker_detected_once = False

        marker_id = goal_handle.request.marker_id
        self.marker_frame = f'ar_marker_{marker_id}'

        self.current_state = self.ParkingSequence.search
        self.current_nearby_sequence = self.NearbySequence.initial_turn

        while rclpy.ok() and not self._done:
            time.sleep(0.1)

        goal_handle.succeed()

        result = Parking.Result()
        result.success = True
        result.message = "Parking completed"

        self.get_logger().info('Action finished successfully')

        return result

    # ================= TIMER =================
    def _run(self):

        if self.current_state == self.ParkingSequence.waiting:
            return

        self.get_logger().info(
            f'[STATE] {self.current_state}, nearby {self.current_nearby_sequence}, marker={self.is_marker_pose_received}'
        )
        

        # self.get_marker_pose()

        if self.current_state == self.ParkingSequence.search:
            self.is_sequence_finished = self.seq_find_goal()
            if self.is_sequence_finished:
                self.get_logger().info('Finished find goal sequence')
                self.is_sequence_finished = False
                self.current_state = self.ParkingSequence.align

        elif self.current_state == self.ParkingSequence.align:
            self.is_sequence_finished = self.seq_align_direction()
            if self.is_sequence_finished:
                self.get_logger().info('align_direction done')
                self.is_sequence_finished = False
                self.current_state = self.ParkingSequence.move

        elif self.current_state == self.ParkingSequence.move:
            self.is_sequence_finished = self.seq_move_nearby_parking_lot()
            print(self.is_sequence_finished)
            if self.is_sequence_finished:
                self.get_logger().info('move_nearby done')
                self.is_sequence_finished = False
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
                self.is_triggered = False

                self.is_sequence_finished = False
                self.is_parking_retried = False

                self.is_odom_received = False
                self.is_marker_pose_received = False

        elif self.current_state == self.ParkingSequence.done:
            self.get_logger().info('Parking Action Succeeded')
            self._done = True
            self.current_state = self.ParkingSequence.waiting
            self.current_nearby_sequence = self.NearbySequence.initial_turn

    # ================= TF =================
    def get_marker_odom(self):
        if self.marker_frame is None:
            return

        result = self.get_2D_marker_pose()
        if result is None:
            return
        pos_x, pos_y, theta = result

        self.marker_2d_pose_x = pos_x
        self.marker_2d_pose_y = pos_y
        self.marker_2d_theta = theta - math.pi

        self.is_marker_pose_received = True

    def get_2D_marker_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(
                'base_link', self.marker_frame, rclpy.time.Time().to_msg())
        except Exception:
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

    # ================= ODOM =================
    def get_robot_odom(self, robot_odom_msg):
        if not self.is_odom_received:
            self.is_odom_received = True
            self.get_logger().info('odom is received!')

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

        if self.is_marker_pose_received:
            if not self.marker_detected_once:
                self.marker_detected_once = True
                self.fn_stop()
                return True

        self.fn_turn(-0.15)
        return False
    
    def seq_align_direction(self):
        desired_angle_turn = -0.5 * math.atan2(self.marker_2d_pose_y - 0, self.marker_2d_pose_x - 0)
        print(f'desired_angle_turn: {desired_angle_turn}')
        self.fn_turn(desired_angle_turn)

        if abs(desired_angle_turn) < 0.01:
            self.fn_stop()
            return True
        else:
            return False

    def seq_move_nearby_parking_lot(self):
        if self.current_nearby_sequence == self.NearbySequence.initial_turn:
            if not self.is_triggered:
                self.is_triggered = True
                self.initial_robot_pose_theta = self.robot_2d_theta
                self.initial_robot_pose_x = self.robot_2d_pose_x
                self.initial_robot_pose_y = self.robot_2d_pose_y
                self.initial_marker_pose_theta = self.marker_2d_theta
                self.initial_marker_pose_x = self.marker_2d_pose_x

            if self.initial_marker_pose_theta < 0.0:
                desired_angle_turn = (math.pi / 2.0) + self.initial_marker_pose_theta - (
                    self.robot_2d_theta - self.initial_robot_pose_theta)
            elif self.initial_marker_pose_theta > 0.0:
                desired_angle_turn = -(math.pi / 2.0) + self.initial_marker_pose_theta - (
                    self.robot_2d_theta - self.initial_robot_pose_theta)
            else:
                desired_angle_turn = 0.0
            desired_angle_turn = -1. * desired_angle_turn

            self.fn_turn(desired_angle_turn)

            if abs(desired_angle_turn) < 0.05:
                self.fn_stop()
                self.current_nearby_sequence = self.NearbySequence.go_straight
                self.is_triggered = False

        elif self.current_nearby_sequence == self.NearbySequence.go_straight:
            dist_from_start = self.calculate_distance_points(
                self.initial_robot_pose_x, self.robot_2d_pose_x,
                self.initial_robot_pose_y, self.robot_2d_pose_y)

            desired_dist = self.initial_marker_pose_x * abs(
                math.cos((math.pi / 2.) - self.initial_marker_pose_theta))
            remained_dist = desired_dist - dist_from_start

            self.fn_go_straight()
            if remained_dist < 0.01:
                self.fn_stop()
                self.current_nearby_sequence = self.NearbySequence.turn_right

        elif self.current_nearby_sequence == self.NearbySequence.turn_right:
            if not self.is_triggered:
                self.is_triggered = True
                self.initial_robot_pose_theta = self.robot_2d_theta

            if self.initial_marker_pose_theta < 0.0:
                desired_angle_turn = -(math.pi / 2.0) + (
                    self.robot_2d_theta - self.initial_robot_pose_theta)
            elif self.initial_marker_pose_theta > 0.0:
                desired_angle_turn = (math.pi / 2.0) + (
                    self.robot_2d_theta - self.initial_robot_pose_theta)
            else:
                desired_angle_turn = 0.0

            self.fn_turn(desired_angle_turn)

            if abs(desired_angle_turn) < 0.05:
                self.fn_stop()
                self.current_nearby_sequence = self.NearbySequence.parking
                self.is_triggered = False
                return True
        return False

    def seq_parking(self):
        desired_angle_turn = math.atan2(self.marker_2d_pose_y - 0, self.marker_2d_pose_x - 0)
        self.fn_track_marker(-desired_angle_turn)
        self.get_logger().info(f'{self.marker_2d_pose_x}')
        if abs(self.marker_2d_pose_x) < self.parking_distance_to_marker:
            self.fn_stop()
            return True
        else:
            return False

    def calculate_distance_points(self, x1, x2, y1, y2):
        return math.sqrt((x1 - x2) ** 2. + (y1 - y2) ** 2.)

    # ================= MOTION =================
    def fn_stop(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist)

    def fn_turn(self, theta):
        Kp = 1.0
        angular_z = Kp * theta

        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = -angular_z
        self.pub_cmd_vel.publish(twist)

    def fn_go_straight(self):
        twist = Twist()
        twist.linear.x = 0.2
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        self.pub_cmd_vel.publish(twist)

    def fn_go_back(self, theta):
        Kp = 0.6

        angular_z = Kp * theta

        twist = Twist()
        twist.linear.x = -0.1
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = -angular_z
        self.pub_cmd_vel.publish(twist)

    def fn_track_marker(self, theta):
        Kp = 0.6

        angular_z = Kp * theta

        twist = Twist()
        twist.linear.x = 0.1
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = -angular_z
        self.pub_cmd_vel.publish(twist)


# ================= MAIN =================
def main(args=None):
    rclpy.init(args=args)

    node = AutomaticParkingVision()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)

    executor.spin()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()