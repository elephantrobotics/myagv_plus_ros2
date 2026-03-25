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

        self.current_state = self.ParkingSequence.waiting

        # ===== TF =====
        self.marker_frame = None
        self.is_marker_pose_received = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ===== ODOM =====
        self.robot_2d_theta = 0.0

        # ===== TIMER =====
        self.timer = self.create_timer(0.1, self._run)

        # ===== ACTION CONTROL =====
        self._goal_handle = None
        self._done = False

    # ================= ACTION =================
    def execute_callback(self, goal_handle):
        self.get_logger().info('Parking Goal Received')

        self._goal_handle = goal_handle
        self._done = False

        marker_id = goal_handle.request.marker_id
        self.marker_frame = f'ar_marker_{marker_id}'

        self.current_state = self.ParkingSequence.search

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
            f'[STATE] {self.current_state}, marker={self.is_marker_pose_received}'
        )

        self.get_marker_pose()

        if self.current_state == self.ParkingSequence.search:
            if not self.is_marker_pose_received:
                self.fn_turn(-0.2)
            else:
                self.get_logger().info('marker detected, switching state')
                self.fn_stop()
                self.current_state = self.ParkingSequence.align

        elif self.current_state == self.ParkingSequence.align:
            self.get_logger().info('align_direction done')
            self.current_state = self.ParkingSequence.move

        elif self.current_state == self.ParkingSequence.move:
            self.get_logger().info('move_nearby done')
            self.current_state = self.ParkingSequence.parking

        elif self.current_state == self.ParkingSequence.parking:
            self.get_logger().info('parking done')
            self.current_state = self.ParkingSequence.done

        elif self.current_state == self.ParkingSequence.done:
            self.get_logger().info('Parking Action Succeeded')
            self._done = True
            self.current_state = self.ParkingSequence.waiting

    # ================= TF =================
    def get_marker_pose(self):
        if self.marker_frame is None:
            return

        try:
            self.tf_buffer.lookup_transform(
                'base_link',
                self.marker_frame,
                rclpy.time.Time()
            )
            self.is_marker_pose_received = True
            self.get_logger().info('[TF] marker detected')
        except Exception:
            self.is_marker_pose_received = False
            self.get_logger().warn('[TF] marker not found')

    # ================= ODOM =================
    def get_robot_odom(self, msg):
        quaternion = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

        r = Rotation.from_quat(quaternion)
        theta = r.as_euler('xyz')[2]

        self.robot_2d_theta = theta

    # ================= DEBUG MOTION =================
    def fn_stop(self):
        self.get_logger().info('[DEBUG] STOP')

    def fn_turn(self, theta):
        self.get_logger().info(f'[DEBUG] TURN theta={theta}')


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