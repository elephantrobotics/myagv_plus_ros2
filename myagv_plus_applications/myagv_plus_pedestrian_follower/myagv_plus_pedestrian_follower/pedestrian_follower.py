#!/usr/bin/env python3

import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from builtin_interfaces.msg import Time
import time

class PedestrianFollower(Node):
    def __init__(self):
        super().__init__('pedestrian_follower')
        
        # 声明参数
        self.declare_parameter("image_topic", "/camera/image_raw")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("min_distance", 2.0)
        self.declare_parameter("max_distance", 5.0)
        self.declare_parameter("max_linear_speed", 0.25)
        self.declare_parameter("max_angular_speed", 1.0)
        self.declare_parameter("frame_id", "base_footprint")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("timeout_duration", 3.0)
        
        # 获取参数
        self.image_topic = self.get_parameter("image_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.min_distance = self.get_parameter("min_distance").value
        self.max_distance = self.get_parameter("max_distance").value
        self.max_linear_speed = self.get_parameter("max_linear_speed").value
        self.max_angular_speed = self.get_parameter("max_angular_speed").value
        self.frame_id = self.get_parameter("frame_id").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.timeout_duration = self.get_parameter("timeout_duration").value
        
        self.bridge = CvBridge()
        
        # HOG行人检测器
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        
        # 订阅相机图像
        self.image_subscriber = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )
        
        # 只发布 TwistStamped（控制器要求）
        self.cmd_vel_publisher = self.create_publisher(
            TwistStamped, self.cmd_vel_topic, 10
        )
        
        # 订阅手势启动信号
        self.start_subscriber = self.create_subscription(
            String, '/start_pedestrian_follow', self.start_callback, 10
        )
        
        # 定时器：定期发布速度命令
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_callback)
        
        # 状态变量
        self.following = False
        self.target_detected = False
        self.last_detection_time = None
        
        # 当前速度命令
        self.current_twist = TwistStamped()
        self.current_twist.header.frame_id = self.frame_id
        self.current_twist.twist.linear.x = 0.0
        self.current_twist.twist.angular.z = 0.0
        
        # 目标历史
        self.target_history = []
        self.max_history_length = 30
        
        self.get_logger().info("=" * 50)
        self.get_logger().info("Pedestrian Follower Node Started")
        self.get_logger().info(f"Subscribing to: {self.image_topic}")
        self.get_logger().info(f"Publishing TwistStamped to: {self.cmd_vel_topic}")
        self.get_logger().info(f"Frame ID: {self.frame_id}")
        self.get_logger().info(f"Publish rate: {self.publish_rate} Hz")
        self.get_logger().info(f"Timeout: {self.timeout_duration} s")
        self.get_logger().info(f"Distance config range: {self.min_distance}m - {self.max_distance}m")
        self.get_logger().info("Waiting for 'start' gesture...")
        self.get_logger().info("=" * 50)
    
    def create_twist_stamped(self, linear_x=0.0, angular_z=0.0):
        """创建带时间戳的 TwistStamped 消息"""
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        twist.header.frame_id = self.frame_id
        twist.twist.linear.x = float(linear_x)
        twist.twist.linear.y = 0.0
        twist.twist.linear.z = 0.0
        twist.twist.angular.x = 0.0
        twist.twist.angular.y = 0.0
        twist.twist.angular.z = float(angular_z)
        return twist
    
    def estimate_distance(self, bounding_box_height, frame_height):
        """距离估算逻辑保留，仅不输出"""
        focal_length = 300.0
        real_height = 1.7
        distance = (real_height * focal_length) / bounding_box_height
        return distance
    
    def calculate_speed(self, distance):
        if distance < self.min_distance:
            linear_speed = max(-0.3, (distance - self.min_distance) * 0.5)
        elif distance > self.max_distance:
            linear_speed = min(self.max_linear_speed, (distance - self.max_distance) * 0.3)
        else:
            linear_speed = self.max_linear_speed * 0.5
        return linear_speed
    
    def calculate_angular_speed(self, center_x, frame_width):
        error = center_x - frame_width / 2
        angular_speed = -error * 0.003
        angular_speed = max(-self.max_angular_speed, min(self.max_angular_speed, angular_speed))
        return angular_speed
    
    def start_callback(self, msg):
        if msg.data == "start":
            self.get_logger().info("Received gesture control start signal!")
            self.following = True
            self.last_detection_time = self.get_clock().now()
            self.get_logger().info("Pedestrian follow STARTED!")
    
    def assign_target_id(self, boxes):
        if not self.target_history:
            if boxes:
                best_box = max(boxes, key=lambda b: b[2] * b[3])
                return best_box
            return None
        
        best_match = None
        min_distance = float('inf')
        
        for box in boxes:
            x, y, w, h = box
            current_center = (x + w//2, y + h//2)
            
            for history_box in self.target_history:
                hx, hy, hw, hh = history_box
                history_center = (hx + hw//2, hy + hh//2)
                dist = ((current_center[0] - history_center[0])**2 + 
                       (current_center[1] - history_center[1])**2)**0.5
                
                if dist < min_distance and dist < 150:
                    min_distance = dist
                    best_match = box
        
        if best_match is None and boxes:
            best_match = max(boxes, key=lambda b: b[2] * b[3])
        
        return best_match
    
    def image_callback(self, msg):
        if not self.following:
            return
            
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame_height, frame_width = frame.shape[:2]
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            boxes, weights = self.hog.detectMultiScale(
                gray, winStride=(8, 8), padding=(16, 16), scale=1.05
            )
            
            self.target_detected = False
            
            if len(boxes) > 0:
                valid_boxes = []
                for i, (x, y, w, h) in enumerate(boxes):
                    confidence = weights[i] if i < len(weights) else 0
                    if confidence > 0.3 and h > w:
                        valid_boxes.append((x, y, w, h))
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                if valid_boxes:
                    self.target_detected = True
                    self.last_detection_time = self.get_clock().now()
                    
                    target_box = self.assign_target_id(valid_boxes)
                    if target_box:
                        x, y, w, h = target_box
                        center_x = x + w // 2
                        
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)
                        
                        self.target_history.append((x, y, w, h))
                        if len(self.target_history) > self.max_history_length:
                            self.target_history.pop(0)
                        
                        distance = self.estimate_distance(h, frame_height)
                        linear_speed = self.calculate_speed(distance)
                        angular_speed = self.calculate_angular_speed(center_x, frame_width)
                        
                        self.current_twist = self.create_twist_stamped(linear_speed, angular_speed)
                        self.get_logger().info(f"Following: linear={linear_speed:.3f}m/s, angular={angular_speed:.3f}rad/s")
                        return
            
            # 无目标时发布零速度
            if not self.target_detected:
                self.current_twist = self.create_twist_stamped(0.0, 0.0)
                
        except Exception as e:
            self.get_logger().error(f"Image processing error: {str(e)}")
            self.current_twist = self.create_twist_stamped(0.0, 0.0)
    
    def timer_callback(self):
        if self.following and self.last_detection_time:
            current_time = self.get_clock().now()
            time_diff = current_time - self.last_detection_time
            if time_diff.nanoseconds / 1e9 > self.timeout_duration:
                self.get_logger().warn("Target lost for too long, stopping robot")
                self.following = False
                self.current_twist = self.create_twist_stamped(0.0, 0.0)
                self.target_history.clear()
        
        # 持续发布速度命令（含零速度）
        if self.following:
            self.cmd_vel_publisher.publish(self.current_twist)
            if abs(self.current_twist.twist.linear.x) > 0.001 or abs(self.current_twist.twist.angular.z) > 0.001:
                self.get_logger().debug(
                    f"Publishing: lin={self.current_twist.twist.linear.x:.3f}, "
                    f"ang={self.current_twist.twist.angular.z:.3f}"
                )
        else:
            zero_twist = self.create_twist_stamped(0.0, 0.0)
            self.cmd_vel_publisher.publish(zero_twist)

def main():
    rclpy.init()
    node = PedestrianFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        zero_twist = TwistStamped()
        zero_twist.header.frame_id = "base_footprint"
        zero_twist.twist.linear.x = 0.0
        zero_twist.twist.angular.z = 0.0
        node.cmd_vel_publisher.publish(zero_twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()