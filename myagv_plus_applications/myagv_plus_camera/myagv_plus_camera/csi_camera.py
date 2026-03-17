#!/usr/bin/env python3

import os
import cv2
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
from ament_index_python.packages import get_package_share_directory
import yaml

class MyAgvPlusCamera(Node):
    def __init__(self):
        super().__init__('myagv_plus_camera')
        self.declare_parameter("video_sensor_id", 0)
        self.declare_parameter("image_width", 1280)
        self.declare_parameter("image_height", 720)
        self.declare_parameter("fps", 30)
        self.declare_parameter("flip_method", 2)
        self.declare_parameter("frame_id", "camera_link")

        self.video_sensor_id = self.get_parameter("video_sensor_id").value
        self.image_width = self.get_parameter("image_width").value
        self.image_height = self.get_parameter("image_height").value
        self.fps = self.get_parameter("fps").value
        self.flip_method = self.get_parameter("flip_method").value
        self.frame_id = self.get_parameter("frame_id").value

        # get the calibration file path from the package share directory
        package_share = get_package_share_directory("myagv_plus_camera")
        self.calibration_path = os.path.join(
            package_share,
            "config",
            "camera.yaml"
        )

        self.calibration_data = self.load_camera_info(self.calibration_path)

        self.get_logger().info(f"Using video sensor ID: {self.video_sensor_id}")
        self.get_logger().info(f"Image width: {self.image_width}")
        self.get_logger().info(f"Image height: {self.image_height}")
        self.get_logger().info(f"FPS: {self.fps}")
        self.get_logger().info(f"Flip method: {self.flip_method}")
        self.get_logger().info(f"Calibration file path: {self.calibration_path}")

        self.publisher_raw = self.create_publisher(
            Image, "/camera/image_raw", 10
        )
        self.publisher_compressed = self.create_publisher(
            CompressedImage, "/camera/image_raw/compressed", 10
        )
        self.publisher_camera_info = self.create_publisher(
            CameraInfo, "/camera/camera_info", 10
        )   

        self.bridge = CvBridge()
        self.video_capture = self.init_csi_capture(
            self.video_sensor_id, self.image_width, self.image_height, self.fps, self.flip_method
        )
        if not self.video_capture.isOpened():
            self.get_logger().error(f"Unable to open video sensor {self.video_sensor_id}")
            return

    def spin_loop(self):
        while rclpy.ok():
            ret, frame = self.video_capture.read()
            if not ret:
                self.get_logger().error("Failed to capture image")
                return
            
            # frame = self.calibrate_camera(frame, self.calibration_data)

            now = self.get_clock().now().to_msg()
            
            # Publish raw image
            raw_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            raw_msg.header.stamp = now
            raw_msg.header.frame_id = self.frame_id
            self.publisher_raw.publish(raw_msg)

            # Publish compressed image
            # compressed_frame = cv2.imencode('.jpg', frame)[1].tobytes()
            # compressed_msg = CompressedImage()
            # compressed_msg.header.stamp = now
            # compressed_msg.header.frame_id = self.frame_id
            # compressed_msg.format = "jpeg"
            # compressed_msg.data = compressed_frame
            # self.publisher_compressed.publish(compressed_msg) 

            if self.calibration_data:
                camera_info = self.calibration_data
                camera_info.header.stamp = now
                camera_info.header.frame_id = self.frame_id
                self.publisher_camera_info.publish(camera_info)

    def init_csi_capture(self, sensor_id, width, height, fps, flip_method):
            gstreamer_pipeline = (
                f"nvarguscamerasrc sensor-id={sensor_id} ! "
                f"video/x-raw(memory:NVMM), width=(int){width}, height=(int){height}, framerate=(fraction){fps}/1 ! "
                f"nvvidconv flip-method={flip_method} ! "
                f"video/x-raw, width=(int){width}, height=(int){height}, format=(string)BGRx ! "
                "videoconvert ! "
                "video/x-raw, format=(string)BGR ! appsink"
            )
            self.get_logger().info(f"GStreamer pipeline: {gstreamer_pipeline}")
            return cv2.VideoCapture(gstreamer_pipeline, cv2.CAP_GSTREAMER)

    def calibrate_camera(self, frame, calibration_data):

        if calibration_data:
            frame = cv2.undistort(
                frame,
                np.array(calibration_data.k).reshape(3, 3),
                np.array(calibration_data.d),
            )
        return frame

    def load_camera_info(self, path):
        if not os.path.exists(path):
            self.get_logger().error(f"Calibration file not found: {path}")
            return None

        with open(path, "r") as f:
            yaml_data = yaml.safe_load(f)

        camera_info_msg = CameraInfo()
        camera_info_msg.height = yaml_data["image_height"]
        camera_info_msg.width = yaml_data["image_width"]
        camera_info_msg.header.frame_id = yaml_data["camera_name"]
        camera_info_msg.distortion_model = yaml_data["distortion_model"]
        camera_info_msg.d = yaml_data["distortion_coefficients"]["data"]
        camera_info_msg.k = yaml_data["camera_matrix"]["data"]
        camera_info_msg.r = yaml_data["rectification_matrix"]["data"]
        camera_info_msg.p = yaml_data["projection_matrix"]["data"]

        return camera_info_msg

def main():
    rclpy.init()
    node = MyAgvPlusCamera()

    try:
        node.spin_loop()
    except KeyboardInterrupt:
        pass
    finally:
        node.video_capture.release()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()