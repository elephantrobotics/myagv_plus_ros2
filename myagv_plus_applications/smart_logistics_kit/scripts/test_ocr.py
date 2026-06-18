#!/usr/bin/env python3

import argparse
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from smart_logistics_kit.OCRVideoCapture import OCRVideoCapture


class OcrTest(Node):
    def __init__(self, topic, interval):
        super().__init__('test_ocr')
        self.bridge = CvBridge()
        self.ocr = OCRVideoCapture()
        self.get_logger().info('OCR model loaded.')
        self.interval = interval
        self.last_run = 0.0
        self.latest_image_msg = None
        self.create_subscription(Image, topic, self.on_image, qos_profile_sensor_data)
        self.create_timer(0.1, self.tick)
        self.get_logger().info(f'Subscribing {topic}, waiting for frames...')

    def on_image(self, msg):
        self.latest_image_msg = msg

    def tick(self):
        if self.latest_image_msg is None:
            return
        now = time.monotonic()
        if now - self.last_run < self.interval:
            return
        self.last_run = now
        try:
            frame = self.bridge.imgmsg_to_cv2(self.latest_image_msg, desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().error(f'frame convert failed: {error}')
            return
        annotated, texts = self.ocr.recognize(frame)
        self.get_logger().info(f'识别文字: {texts}')
        cv2.imshow('OCR Test', annotated)
        cv2.waitKey(1)


def main():
    parser = argparse.ArgumentParser(description='OCR 话题识别测试')
    parser.add_argument('--topic', default='/camera/image_raw')
    parser.add_argument('--interval', type=float, default=0.5)
    args = parser.parse_args()

    rclpy.init()
    node = OcrTest(args.topic, args.interval)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
