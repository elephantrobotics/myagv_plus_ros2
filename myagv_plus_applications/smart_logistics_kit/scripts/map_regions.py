#!/usr/bin/env python3

import math
import time
from pathlib import Path

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
import yaml

from smart_logistics_kit.OCRVideoCapture import OCRVideoCapture


GREEN = '\033[32m'
YELLOW = '\033[33m'
RED = '\033[31m'
RESET = '\033[0m'

DEFAULT_REGIONS = ['华北区', '华东区', '华南区', '东北区', '华中区']
SCAN_WAYPOINTS = ['A', 'B', 'C', 'D', 'E']
MAP_ORIGIN = [0.0, 0.0, 0.0, 1.0]  # 建图原点位姿 (x, y, z, w)；Map origin pose.
MATCH_TIMEOUT = 10.0


class RegionMapper(Node):
    def __init__(self):
        super().__init__('region_mapper')

        self.bridge_final = {'route_bridge_succeeded', 'route_bridge_failed', 'route_bridge_canceled'}
        self.refiner_failed = {'timeout', 'failed_tf', 'no_goal', 'handoff_distance_exceeded', 'canceled'}

        scripts_dir = Path(__file__).resolve().parent
        self.waypoints_file = str(scripts_dir / 'waypoints.yaml')
        self.camera_image_topic = '/camera/image_raw'

        self.goal_pose_pub = self.create_publisher(PoseStamped, 'route_bridge/goal_pose', 10)
        self.create_subscription(String, 'route_bridge/feedback', self.bridge_feedback_cb, 10)
        self.create_subscription(String, '/final_pose_refiner/status', self.refiner_status_cb, 10)
        self.create_subscription(Image, self.camera_image_topic, self.camera_image_cb, qos_profile_sensor_data)

        self.bridge_feedback = None
        self.refiner_seq = 0
        self.refiner_history = []
        self.latest_image_msg = None
        self.cv_bridge = CvBridge()

        self.waypoints = self.load_waypoints(self.waypoints_file)
        self.ocr = OCRVideoCapture()
        self.get_logger().info(f'{GREEN}OCR model loaded.{RESET}')
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {self.waypoints_file}')

    def camera_image_cb(self, msg):
        self.latest_image_msg = msg

    def bridge_feedback_cb(self, msg):
        self.bridge_feedback = msg.data
        self.get_logger().info(f'route_bridge feedback: {msg.data}')

    def refiner_status_cb(self, msg):
        status = msg.data.strip()
        if status:
            self.refiner_seq += 1
            self.refiner_history.append((self.refiner_seq, status))
            self.refiner_history = self.refiner_history[-32:]

    def load_waypoints(self, waypoints_file):
        path = Path(waypoints_file)
        data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
        waypoints = {}
        for key, value in (data.get('waypoints') or {}).items():
            name = str(key).strip().upper()
            waypoints[name] = [float(item) for item in value]
        return waypoints

    def make_pose(self, waypoint):
        return self.make_pose_from_coords(self.waypoints[waypoint], waypoint)

    def make_pose_from_coords(self, coords, label):
        x, y, z, w = coords
        norm = math.hypot(z, w)
        if norm < 1e-6:
            raise RuntimeError(f'{label} has invalid orientation: z={z}, w={w}')
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.z, pose.pose.orientation.w = z / norm, w / norm
        return pose

    def wait_for_bridge(self):
        self.get_logger().info('Waiting for route_bridge subscriber on /route_bridge/goal_pose.')
        while rclpy.ok():
            if self.goal_pose_pub.get_subscription_count() > 0:
                return True
            time.sleep(0.1)
        return False

    def wait_for_bridge_result(self):
        deadline = time.monotonic() + 600.0
        while rclpy.ok() and time.monotonic() < deadline:
            if self.bridge_feedback in self.bridge_final:
                return self.bridge_feedback == 'route_bridge_succeeded'
            time.sleep(0.1)
        self.get_logger().warn('No final route_bridge feedback within 600.0 s.')
        return False

    def wait_for_refiner(self, start_seq):
        deadline = time.monotonic() + 30.0
        saw_running = False
        sequence = start_seq
        while rclpy.ok() and time.monotonic() < deadline:
            updates = [(seq, status) for seq, status in self.refiner_history if seq > sequence]
            for sequence, status in updates:
                if status == 'running':
                    saw_running = True
                elif status == 'succeeded' and saw_running:
                    self.get_logger().info('Final pose refinement succeeded.')
                    return True
                elif status in self.refiner_failed and (
                    saw_running or status in {'failed_tf', 'no_goal', 'handoff_distance_exceeded'}
                ):
                    self.get_logger().error(f'Final pose refinement failed: {status}.')
                    return False
            time.sleep(0.1)
        self.get_logger().warn('No final pose refinement success within 30.0 s.')
        return False

    def navigate_to_pose(self, pose, label):
        if not self.wait_for_bridge():
            return False
        start_seq = self.refiner_seq
        self.bridge_feedback = None
        self.goal_pose_pub.publish(pose)
        self.get_logger().info(f'Sent route_bridge goal for {label}.')
        return self.wait_for_bridge_result() and self.wait_for_refiner(start_seq)

    def navigate_to_waypoint(self, waypoint):
        if waypoint not in self.waypoints:
            self.get_logger().warn(f'Waypoint {waypoint} does not exist.')
            return False
        return self.navigate_to_pose(self.make_pose(waypoint), f'waypoint {waypoint}')

    def navigate_to_origin(self):
        pose = self.make_pose_from_coords(MAP_ORIGIN, 'map origin')
        return self.navigate_to_pose(pose, 'map origin (0,0)')

    def grab_frame(self):
        if self.latest_image_msg is None:
            return None
        try:
            return self.cv_bridge.imgmsg_to_cv2(self.latest_image_msg, desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().error(f'frame convert failed: {error}')
            return None

    def match_region_at(self, waypoint, taken_regions):
        self.get_logger().info(f'Scanning region label at waypoint {waypoint} (timeout {MATCH_TIMEOUT:.0f}s).')
        deadline = time.monotonic() + MATCH_TIMEOUT
        last_warn = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            frame = self.grab_frame()
            if frame is None:
                time.sleep(0.05)
                continue
            annotated, texts = self.ocr.recognize(frame)
            cv2.imshow('Region Mapping', annotated)
            cv2.waitKey(1)
            joined = ''.join(texts)
            for region in DEFAULT_REGIONS:
                if region in joined and region not in taken_regions:
                    self.get_logger().info(f'{GREEN}{waypoint} 点匹配分区: {region}{RESET}')
                    hold_until = time.monotonic() + 1.0
                    while rclpy.ok() and time.monotonic() < hold_until:
                        cv2.imshow('Region Mapping', annotated)
                        cv2.waitKey(1)
                        time.sleep(0.03)
                    return region
            now = time.monotonic()
            if now - last_warn >= 1.0:
                last_warn = now
                print(f'{YELLOW}该 {waypoint} 点未放置分区，继续识别中...{RESET}')
            time.sleep(0.05)
        print(f'{RED}该 {waypoint} 点超过 {MATCH_TIMEOUT:.0f}s 未匹配到分区，跳过该点。{RESET}')
        return None

    def write_region_yaml(self, region_to_waypoint, final=False, merge=False):
        path = Path(self.waypoints_file)
        data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
        if merge:
            result = dict(data.get('region_to_waypoint') or {})
            used_waypoints = set(region_to_waypoint.values())
            result = {region: waypoint for region, waypoint in result.items()
                      if waypoint not in used_waypoints and region not in region_to_waypoint}
            result.update(region_to_waypoint)
        else:
            result = dict(region_to_waypoint)
        ordered = {region: result[region]
                   for region in DEFAULT_REGIONS if region in result}
        ordered.update({region: waypoint for region, waypoint in result.items()
                        if region not in ordered})
        data['region_to_waypoint'] = ordered
        path.write_text(
            yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding='utf-8')
        tag = '最终' if final else '增量'
        self.get_logger().info(f'{GREEN}已{tag}写入分区映射 -> {self.waypoints_file}{RESET}')

    def run(self, scan_waypoints, auto_fill):
        region_to_waypoint = {}
        taken_regions = set()
        skipped = []
        merge = not auto_fill

        if not self.navigate_to_origin():
            self.get_logger().error('Failed to reach map origin. Abort mapping.')
            return

        for waypoint in scan_waypoints:
            if not self.navigate_to_waypoint(waypoint):
                self.get_logger().error(f'Failed to reach waypoint {waypoint}; treat as skipped.')
                skipped.append(waypoint)
                continue
            region = self.match_region_at(waypoint, taken_regions)
            cv2.destroyAllWindows()
            cv2.waitKey(1)
            if region is None:
                skipped.append(waypoint)
                continue
            region_to_waypoint[region] = waypoint
            taken_regions.add(region)
            self.write_region_yaml(region_to_waypoint, final=False, merge=merge)

        if auto_fill:
            remaining_regions = [r for r in DEFAULT_REGIONS if r not in taken_regions]
            if skipped or remaining_regions:
                self.get_logger().warn(
                    f'跳过点位: {skipped or "无"}; 未分配分区: {remaining_regions or "无"}。'
                    '开始为跳过点位补齐剩余分区。')
            for waypoint, region in zip(skipped, remaining_regions):
                region_to_waypoint[region] = waypoint
                self.get_logger().info(f'{YELLOW}补齐: {region} -> {waypoint}{RESET}')
        elif skipped:
            self.get_logger().warn(f'跳过点位: {skipped}; 指定模式不自动补齐，这些点保持原有映射。')

        self.write_region_yaml(region_to_waypoint, final=True, merge=merge)
        self.get_logger().info(f'{GREEN}分区映射完成: {region_to_waypoint}{RESET}')

        self.navigate_to_origin()


def parse_scan_waypoints(argv):
    if len(argv) <= 1:
        return list(SCAN_WAYPOINTS), True
    arg = argv[1].strip().upper()
    waypoints = list(arg)
    if not waypoints or any(w not in SCAN_WAYPOINTS for w in waypoints):
        print(f'{RED}Invalid argument: "{argv[1]}". Only A-E combinations are accepted, e.g. A / AB / ABCD.{RESET}')
        return None, False
    if len(set(waypoints)) != len(waypoints):
        print(f'{RED}Invalid argument: "{argv[1]}" contains duplicate waypoints.{RESET}')
        return None, False
    auto_fill = set(waypoints) == set(SCAN_WAYPOINTS)
    return waypoints, auto_fill


def main():
    import sys
    scan_waypoints, auto_fill = parse_scan_waypoints(sys.argv)
    if scan_waypoints is None:
        return

    rclpy.init()
    node = RegionMapper()
    import threading
    worker = threading.Thread(target=node.run, args=(scan_waypoints, auto_fill), daemon=True)
    worker.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
