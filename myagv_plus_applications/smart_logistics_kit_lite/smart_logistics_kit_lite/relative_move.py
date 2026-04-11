#!/usr/bin/env python3
import time
import math
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy

class RelativeMoveController:

    def __init__(self,node):

        self.node = node

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_yaw = 0.0

        self.step = 0
        self.active = False
        self.done = False

        self.cmd_pub = self.node.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.node.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        
        self.timer = self.node.create_timer(0.01, self.control_loop)

    def odom_callback(self, msg):
        self.last_pose_x = msg.pose.pose.position.x
        self.last_pose_y = msg.pose.pose.position.y

        _, _, self.last_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)

    def move(self, dx, dy, dtheta, timeout=10.0):

        start_x = self.x
        start_y = self.y
        start_yaw = self.yaw

        self.goal_x = start_x + math.cos(start_yaw)*dx - math.sin(start_yaw)*dy
        self.goal_y = start_y + math.sin(start_yaw)*dx + math.cos(start_yaw)*dy
        self.goal_yaw = start_yaw + dtheta

        self.node.get_logger().info(
            f"[RelativeMove] goal=({self.goal_x:.3f},{self.goal_y:.3f},{self.goal_yaw:.3f})"
        )

        self.step = 1
        self.active = True
        self.done = False

        start_time = self.node.get_clock().now()

        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.01)
 
            if self.done:
                break

            now = self.node.get_clock().now()
            if (now - start_time).nanoseconds / 1e9 > timeout:
                self.node.get_logger().warn("Relative move timeout")
                break

        self.cmd_pub.publish(Twist())

        return self.done

    def control_loop(self):
        if not self.active:
            return

        twist = Twist()

        dx = self.goal_x - self.x
        dy = self.goal_y - self.y
        dist = math.sqrt(dx*dx + dy*dy)

        target_theta = math.atan2(dy, dx)
        angle_err = self.normalize_angle(target_theta - self.yaw)

        if self.step == 1:
            if abs(angle_err) > 0.01:
                twist.angular.z = 0.3 if angle_err > 0 else -0.3
            else:
                self.step = 2

        elif self.step == 2:
            if dist > 0.02:
                twist.linear.x = 0.15
            else:
                self.step = 3

        elif self.step == 3:
            final_err = self.normalize_angle(self.goal_yaw - self.yaw)
            if abs(final_err) > 0.01:
                twist.angular.z = 0.3 if final_err > 0 else -0.3
            else:
                self.step = 4

        elif self.step == 4:
            self.active = False
            self.done = True
            self.node.get_logger().info("Relative move done")

        self.cmd_pub.publish(twist)


    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def euler_from_quaternion(self, quat):
        x = quat.x
        y = quat.y
        z = quat.z
        w = quat.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = numpy.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = numpy.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = numpy.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw