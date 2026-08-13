#!/usr/bin/env python3

import warnings

warnings.filterwarnings('ignore', message='The value of the smallest subnormal.*', category=UserWarning)

import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
    qos_profile_sensor_data,
)
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


GREEN = '\033[32m'
RED = '\033[31m'
RESET = '\033[0m'


class ArUcoTracker(Node):

    def __init__(self):
        super().__init__('aruco_tracker')

        self.sub_camera_info = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        image_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.sub_camera_image = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            image_qos
        )

        detect_qos = QoSProfile(depth=1)
        detect_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.detect_enable = True
        self.create_subscription(Bool, 'parking/detect_enable', self.detect_enable_cb, detect_qos)

        self.declare_parameter('marker_size', 0.04)
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value
        self.declare_parameter('debug', False)
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        self.declare_parameter('marker_log_interval', 1.0)
        self.marker_log_interval = (
            self.get_parameter('marker_log_interval').get_parameter_value().double_value)

        self.camera_info_msg = None
        self.intrinsic_mat = None
        self.distortion = None
        self.last_marker_log_time = {}

        marker_dict_id = 'DICT_6X6_250'
        dict_id = getattr(cv2.aruco, marker_dict_id)
        self.marker_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.ar_param = cv2.aruco.DetectorParameters_create()
        else:
            self.ar_param = cv2.aruco.DetectorParameters()

        self.bridge = CvBridge()
        self.br = TransformBroadcaster(self)

        self.camera_ready = False  # 收到相机图像+内参后才算就绪；Ready only after image+info received.
        self.last_camera_wait_log = 0.0  # 相机未就绪红字节流时间；Camera-not-ready log throttle.
        self.create_timer(1.0, self.camera_watchdog_cb)  # 相机未就绪每秒红字；Red reminder while camera not ready.
        self.get_logger().info('ArUco Tracker node started. Waiting for camera stream...')

    def camera_watchdog_cb(self):
        if self.camera_ready:
            return
        now = time.monotonic()
        if now - self.last_camera_wait_log >= 1.0:
            self.last_camera_wait_log = now
            missing = 'camera_info' if self.camera_info_msg is None else 'image_raw'
            print(f"{RED}ArUco Tracker not ready: waiting for /camera/{missing} "
                  f"(camera driver down or not connected?).{RESET}")

    def camera_info_callback(self, camera_info_msg):
        self.camera_info_msg = camera_info_msg
        self.intrinsic_mat = np.reshape(np.array(self.camera_info_msg.k), (3, 3))
        self.distortion = np.array(self.camera_info_msg.d)
        self.destroy_subscription(self.sub_camera_info)

    def detect_enable_cb(self, msg):
        self.detect_enable = bool(msg.data)

    def image_callback(self, img_msg: Image):
        if self.camera_info_msg is None:
            return

        if not self.camera_ready:
            self.camera_ready = True
            print(f"{GREEN}ArUco Tracker initialized: camera stream and camera_info received.{RESET}")

        if not self.detect_enable:
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        cv_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        self.tracking_markers(cv_gray, cv_image)

    def tracking_markers(self, cv_gray, cv_image=None):
        corners, marker_ids, _ = cv2.aruco.detectMarkers(
            cv_gray, self.marker_dict, parameters=self.ar_param
        )
        if marker_ids is None:
            if self.debug and cv_image is not None:
                cv2.imshow('aruco_tracker_debug', cv_image)
                cv2.waitKey(1)
            return

        if self.debug and cv_image is not None:
            cv2.aruco.drawDetectedMarkers(cv_image, corners, marker_ids)
            for marker_corners in corners:
                pts = marker_corners.reshape(4, 2).astype(int)
                for index, (x, y) in enumerate(pts):
                    cv2.circle(cv_image, (x, y), 5, (0, 0, 255), -1)
                    cv2.putText(
                        cv_image, str(index), (x + 5, y - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            cv2.imshow('aruco_tracker_debug', cv_image)
            cv2.waitKey(1)

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.intrinsic_mat, self.distortion
        )
        R_corr = Rotation.from_euler('xyz', [-np.pi/2, 0, -np.pi/2]).as_matrix()
        transform_stamp = self.get_clock().now().to_msg()

        for i, marker_id in enumerate(marker_ids.flatten()):
            tvec = tvecs[i][0]
            rvec = rvecs[i][0]

            R_marker, _ = cv2.Rodrigues(rvec)
            R_corrected = R_corr @ R_marker
            t_corrected = R_corr @ tvec
            quat = Rotation.from_matrix(R_corrected).as_quat()

            t_msg = TransformStamped()
            # Parking evaluates freshness when detection finishes; using the
            # capture stamp makes a newly published transform look stale after
            # transport and image-processing latency.
            t_msg.header.stamp = transform_stamp
            t_msg.header.frame_id = 'camera_link'
            t_msg.child_frame_id = f'ar_marker_{marker_id}'
            t_msg.transform.translation.x = float(t_corrected[0])
            t_msg.transform.translation.y = float(t_corrected[1])
            t_msg.transform.translation.z = float(t_corrected[2])
            t_msg.transform.rotation.x = float(quat[0])
            t_msg.transform.rotation.y = float(quat[1])
            t_msg.transform.rotation.z = float(quat[2])
            t_msg.transform.rotation.w = float(quat[3])

            self.br.sendTransform(t_msg)
            self.log_marker_if_active(marker_id)

    def log_marker_if_active(self, marker_id):
        now = self.get_clock().now().nanoseconds / 1e9
        last_log_time = self.last_marker_log_time.get(marker_id, 0.0)
        if now - last_log_time < self.marker_log_interval:
            return

        self.last_marker_log_time[marker_id] = now
        self.get_logger().info(f'ar_marker_{marker_id}')


def main():
    rclpy.init()
    node = ArUcoTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
