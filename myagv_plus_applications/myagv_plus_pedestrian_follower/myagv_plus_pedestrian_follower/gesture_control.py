#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
集成摄像头和手势控制功能
- 提供CSI摄像头图像发布
- 通过五指张开手势启动行人跟随
"""
import threading
import time
import cv2
from cvzone.HandTrackingModule import HandDetector
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# CSI 摄像头专用 GStreamer 管道
def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1280,
    capture_height=720,
    framerate=30,
    flip_method=0,
):
    return (
        "nvarguscamerasrc sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, format=(string)BGRx ! videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink drop=1"
        % (sensor_id, capture_width, capture_height, framerate, flip_method)
    )

# 集成摄像头和手势控制的ROS2节点
class GestureCameraNode(Node):
    def __init__(self):
        super().__init__('gesture_camera_node')
        
        # 摄像头参数
        self.declare_parameter("sensor_id", 0)
        self.declare_parameter("capture_width", 1280)
        self.declare_parameter("capture_height", 720)
        self.declare_parameter("framerate", 30)
        self.declare_parameter("flip_method", 2)
        self.declare_parameter("frame_id", "camera_link")
        
        # 读取参数
        self.sensor_id = self.get_parameter("sensor_id").value
        self.capture_width = self.get_parameter("capture_width").value
        self.capture_height = self.get_parameter("capture_height").value
        self.framerate = self.get_parameter("framerate").value
        self.flip_method = self.get_parameter("flip_method").value
        self.frame_id = self.get_parameter("frame_id").value
        
        # ROS2发布器
        self.bridge = CvBridge()
        self.image_publisher = self.create_publisher(Image, "/camera/image_raw", 10)
        self.start_publisher = self.create_publisher(String, '/start_pedestrian_follow', 10)
        
        # 手势检测相关
        self.detector = HandDetector(
            detectionCon=0.8,  # 检测置信度阈值
            maxHands=2  # 最多检测的手部数量
        )
        self.open_hand_count = 0
        self.open_hand_threshold = 3  # 需要连续检测3帧
        
        # 行人检测相关
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        
        # 初始化CSI摄像头
        pipeline = gstreamer_pipeline(
            sensor_id=self.sensor_id,
            capture_width=self.capture_width,
            capture_height=self.capture_height,
            framerate=self.framerate,
            flip_method=self.flip_method
        )
        self.video_capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        
        # 检查摄像头是否成功打开
        if not self.video_capture.isOpened():
            self.get_logger().error("错误：无法打开CSI摄像头！")
            return
        
        self.get_logger().info("CSI摄像头初始化成功")
        self.get_logger().info("手势控制启动行人跟随功能")
        self.get_logger().info("请挥手")
        
        # 定时处理图像
        self.create_timer(1.0 / self.framerate, self.timer_callback)
    
    def timer_callback(self):
        ret, frame = self.video_capture.read()
        if not ret:
            self.get_logger().warn("未获取到画面")
            return
        
        # 调整图像大小
        frame_resized = cv2.resize(frame, (640, 480))
        
        # 检测手部
        hands, img = self.detector.findHands(frame_resized)  # hands 包含检测结果

        if hands:
            hand_opened = False
            for hand in hands:
                fingers = self.detector.fingersUp(hand)
                if fingers == [1, 1, 1, 1, 1]:
                    hand_opened = True
                    break
            
            if hand_opened:
                self.open_hand_count += 1
                # 显示检测进度
                progress = min(100, (self.open_hand_count / self.open_hand_threshold) * 100)
                cv2.putText(img, f"Open hand: {int(progress)}%", 
                           (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                if self.open_hand_count >= self.open_hand_threshold:
                    # 发布启动消息
                    msg = String()
                    msg.data = "start"
                    self.start_publisher.publish(msg)
                    self.get_logger().info("Pedestrian follow started!")
                    cv2.putText(img, "Follow started!", 
                               (10, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                # 重置计数
                self.open_hand_count = 0
                cv2.putText(img, "Open hand to start", 
                           (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        else:
            # 重置计数
            self.open_hand_count = 0
            cv2.putText(img, "Open hand to start", 
                       (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # 行人检测
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        boxes, weights = self.hog.detectMultiScale(gray, winStride=(8, 8), padding=(16, 16), scale=1.05)
        
        if len(boxes) > 0:
            for i, (x, y, w, h) in enumerate(boxes):
                confidence = weights[i] if i < len(weights) else 0
                if confidence > 0.5 and h > w:
                    # 绘制行人检测框（绿色）
                    cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # 发布图像
        img_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
        img_msg.header.stamp = self.get_clock().now().to_msg()
        img_msg.header.frame_id = self.frame_id
        self.image_publisher.publish(img_msg)
        
        # 显示画面
        cv2.imshow("Gesture Camera Control", img)
        cv2.waitKey(1)
    
    def destroy_node(self):
        self.video_capture.release()
        cv2.destroyAllWindows()
        super().destroy_node()

# 主函数
def main():
    rclpy.init()
    node = GestureCameraNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
