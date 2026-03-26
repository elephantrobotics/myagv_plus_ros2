#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.duration import Duration

from rclpy.action import ActionClient
from myagv_plus_msgs.action import Parking

import time
from smart_logistics_kit_lite.arm_controller import MechArm270Control

class LogisticsMission(Node):
    def __init__(self):
        super().__init__('logistics_mission')

        self.navigator = BasicNavigator()

        # parking control action client
        self.parking_client = ActionClient(self, Parking, 'parking_action')

        # # arm init
        self.ma = MechArm270Control('/dev/ttyACM0',115200)

        self.pub_cmd_vel = self.create_publisher(
            Twist,
            '/cmd_vel',
            qos_profile=QoSProfile(depth=10)
        )

    def call_parking(self, marker_id):

        goal_msg = Parking.Goal()
        goal_msg.marker_id = marker_id

        self.parking_client.wait_for_server()

        self.get_logger().info(f"Calling parking action server with marker_id: {marker_id}")
        send_goal_future = self.parking_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        print("Goal handle received from parking action server")

        if not goal_handle.accepted:
            self.get_logger().error("Parking goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result

        self.get_logger().info(f"Parking result: {result.message}")
        return result.success

    def create_pose(self, x, y, z, w):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.navigator.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        return pose

    def set_initial_pose(self, x: float, y: float, oz: float, ow: float):
        """
        Set the initial pose of the robot for AMCL localization.

        Args:
            navigator (BasicNavigator): The navigator instance controlling the robot.
            x (float): Initial X position in the map frame.
            y (float): Initial Y position in the map frame.
            oz (float): Orientation Z component (quaternion).
            ow (float): Orientation W component (quaternion).
        """
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = x
        initial_pose.pose.position.y = y
        initial_pose.pose.orientation.z = oz
        initial_pose.pose.orientation.w = ow
        self.navigator.setInitialPose(initial_pose)

    def nav_waypoint_follower(self, goal_poses, verbose: bool = False) -> bool:
        
        nav_start = self.navigator.get_clock().now()
        self.navigator.followWaypoints(goal_poses)

        i = 0
        while not self.navigator.isTaskComplete():
            # Do something with the feedback
            i = i + 1
            feedback = self.navigator.getFeedback()
            if (feedback and i % 5) and verbose == 0:
                print('Executing current waypoint: ' +
                    str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
                now = self.navigator.get_clock().now()

                # Some navigation timeout to demo cancellation
                if now - nav_start > Duration(seconds=600.0):
                    self.navigator.cancelTask()

        # Do something depending on the return code
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
            return True
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
        return False
    
    def navigate_to_goal(self, x: float, y: float, oz: float, ow: float, verbose: bool = False) -> bool:
        """
        Navigate the robot to a target goal pose.

        Args:
            navigator (BasicNavigator): The navigator instance controlling the robot.
            x (float): Goal X position in the map frame.
            y (float): Goal Y position in the map frame.
            oz (float): Orientation Z component (quaternion).
            ow (float): Orientation W component (quaternion).
            verbose (bool, optional): If True, prints navigation feedback such as estimated arrival time. Default is False.

        Returns:
            bool: True if navigation succeeded, False otherwise.
        """
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.z = oz
        goal_pose.pose.orientation.w = ow

        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback and verbose:
                remaining = Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9
                print(f"Estimated time of arrival: {remaining:.0f} seconds")

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print('Goal succeeded!')
            return True
        elif result == TaskResult.CANCELED:
            print('Goal was canceled!')
        elif result == TaskResult.FAILED:
            print('Goal failed!')
        else:
            print('Goal has an invalid return status!')
        return False
    
    def pub_vel(self, x, y , theta):
        twist = Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = theta
        self.pub_cmd_vel.publish(twist)

    def run(self):
        # Set robot initial pose
        # self.set_initial_pose(x=0.0, y=0.0, oz=0.0, ow=1.0)

        # Wait for navigation to fully activate, since autostarting nav2
        # self.navigator.waitUntilNav2Active()

        goal_A = [0.67125,0.0014235,-0.0079793, 0.99997]
        goal_B = [0.084248,-0.16886,-0.68813,0.72558]

        for i in range(2):
            x_goal, y_goal, orientation_z, orientation_w = goal_A
            success = self.navigate_to_goal(x_goal, y_goal, orientation_z, orientation_w)
            print("Navigation result:", success)

            if not success:
                return

            self.call_parking(marker_id=4)

            if i == 0:
                box_height = 50
            elif i == 1:
                box_height = 50

            self.ma.pick(box_height)

            x_goal, y_goal, orientation_z, orientation_w = goal_B
            success = self.navigate_to_goal(x_goal, y_goal, orientation_z, orientation_w)
            print("Navigation result:", success)

            self.call_parking(marker_id=5)

            self.ma.place()

def main(args=None):
    rclpy.init(args=args)
    node = LogisticsMission()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()