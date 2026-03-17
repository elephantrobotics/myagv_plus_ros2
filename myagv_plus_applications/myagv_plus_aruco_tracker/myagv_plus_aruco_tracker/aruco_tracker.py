#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
from tf2_ros import TransformBroadcaster


class ArUcoTracker(Node):

    def __init__(self):
        super().__init__('aruco_tracker')

        self.sub_camera_info = self.create_subscription(
            CameraInfo,
            '/camera/camera_info',
            self.camera_info_callback,
            qos_profile_sensor_data
        )

        self.sub_camera_image = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            qos_profile_sensor_data
        )

        self.declare_parameter('marker_size', 0.03)
        self.marker_size = self.get_parameter('marker_size').get_parameter_value().double_value

        self.camera_info_msg = None
        self.intrinsic_mat = None
        self.distortion = None

        marker_dict_id = 'DICT_6X6_250'
        dict_id = getattr(cv2.aruco, marker_dict_id)
        self.marker_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.ar_param = cv2.aruco.DetectorParameters_create()
        else:
            self.ar_param = cv2.aruco.DetectorParameters()

        self.bridge = CvBridge()
        self.br = TransformBroadcaster(self)

        print("ArUco Tracker Node initialized successfully.")

    def camera_info_callback(self, camera_info_msg):
        self.camera_info_msg = camera_info_msg
        self.intrinsic_mat = np.reshape(np.array(self.camera_info_msg.k), (3, 3))
        self.distortion = np.array(self.camera_info_msg.d)
        self.destroy_subscription(self.sub_camera_info)

    def image_callback(self, img_msg: Image):
        if self.camera_info_msg is None:
            self.get_logger().warn('No camera info')
            print("No camera info")
            return

        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        cv_gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        self.tracking_markers(cv_gray, img_msg.header.stamp)

    def tracking_markers(self, cv_gray, stamp):
        corners, marker_ids, _ = cv2.aruco.detectMarkers(
            cv_gray, self.marker_dict, parameters=self.ar_param
        )
        if marker_ids is None:
            print("No markers detected")
            return

        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, self.marker_size, self.intrinsic_mat, self.distortion
        )
        R_corr = Rotation.from_euler('xyz', [-np.pi/2, 0, -np.pi/2]).as_matrix()

        for i, marker_id in enumerate(marker_ids.flatten()):
            tvec = tvecs[i][0]
            rvec = rvecs[i][0]

            R_marker, _ = cv2.Rodrigues(rvec)
            R_corrected = R_corr @ R_marker
            t_corrected = R_corr @ tvec
            quat = Rotation.from_matrix(R_corrected).as_quat()

            t_msg = TransformStamped()
            t_msg.header.stamp = stamp
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
            self.get_logger().info(f'ar_marker_{marker_id}')


def main():
    rclpy.init()
    node = ArUcoTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()