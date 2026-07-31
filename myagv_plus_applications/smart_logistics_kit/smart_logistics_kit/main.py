#!/usr/bin/env python3

from pathlib import Path
import math
import sys
import threading
import time

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, TwistStamped
from myagv_plus_msgs.action import Parking
import rclpy
from rcl_interfaces.msg import Parameter as ParameterMsg
from rcl_interfaces.msg import ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
import yaml


GREEN = '\033[32m'
RED = '\033[31m'
RESET = '\033[0m'


class LogisticsRouteMission(Node):
    def __init__(self):
        super().__init__('logistics_route_mission')

        self.city_to_region = {  # 二维码城市关键字到物流分区；QR city keywords to logistics regions.
            '北京市': '华北区',
            '上海市': '华东区',
            '南京市': '华东区',
            '东莞市': '华南区',
            '广州市': '华南区',
            '武汉市': '华中区',
            '大连市': '东北区',
        }
        self.default_region_to_waypoint = {  # 分区到导航点位默认值；Default region to navigation waypoint.
            '华北区': 'A',
            '华东区': 'B',
            '华南区': 'C',
            '华中区': 'D',
            '东北区': 'E',
        }
        self.bridge_final = {'route_bridge_succeeded', 'route_bridge_failed', 'route_bridge_canceled'}  # route_bridge 最终状态；route_bridge terminal states.
        self.refiner_failed = {'timeout', 'failed_tf', 'no_goal', 'handoff_distance_exceeded', 'canceled'}  # 精定位失败状态；final_pose_refiner failure states.

        workspace_dir = Path(get_package_share_directory('smart_logistics_kit')).resolve().parents[3]
        self.waypoints_file = str(
            workspace_dir / 'src' / 'myagv_plus_applications' /
            'smart_logistics_kit' / 'scripts' / 'waypoints.yaml')  # 导航点位与分区映射文件路径；Waypoint and region mapping YAML path.
        self.pickup_waypoint = 'O'  # 取货点位；Pickup waypoint.
        self.autocharge_waypoint = 'P'  # 回充点位；Autocharge waypoint.
        self.arm_fallback_height = 92.0  # 视觉高度不可用时的兜底高抓；Low packages are selected by QR vision.
        self.retreat_distance = 0.08  # 机械臂取/放后后退距离，单位米；Retreat distance after pick/place, in meters.
        self.retreat_speed = 0.15  # 机械臂取/放后后退速度，单位 m/s；Retreat speed after pick/place, in m/s.
        self.arm_port = self.resolve_arm_port()  # 机械臂串口选择；Arm serial port selection.
        self.arm_baudrate = 115200  # 机械臂串口波特率；Arm serial baudrate.
        self.qr_camera = '/dev/video1'  # 二维码相机设备；QR camera device.
        self.qr_timeout = 50.0  # 二维码识别超时，单位秒；QR scan timeout, in seconds.
        self.camera_image_topic = '/camera/image_raw'  # aruco 相机在线判定话题；aruco camera liveness topic.
        self.camera_fresh_seconds = 2.0  # 相机数据新鲜判定窗口，单位秒；Camera liveness window, in seconds.

        self.goal_pose_pub = self.create_publisher(PoseStamped, 'route_bridge/goal_pose', 10)  # route_bridge 目标点；route_bridge goal topic.
        self.route_cancel_pub = self.create_publisher(Bool, 'route_bridge/cancel', 10)  # route_bridge 取消；route_bridge cancel topic.
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)  # 底盘速度控制；base velocity command topic.
        self.autocharge_ready_pub = self.create_publisher(String, 'autocharge/ready', 10)  # 到达 P 点后触发对接；trigger docking after reaching P.
        self.refiner_param_client = self.create_client(SetParameters, '/final_pose_refiner/set_parameters')  # 精修取消参数服务；final refiner cancel parameter service.
        self.create_subscription(String, 'route_bridge/feedback', self.bridge_feedback_cb, 10)  # 导航状态反馈；navigation feedback.
        self.create_subscription(String, '/final_pose_refiner/status', self.refiner_status_cb, 10)  # 最终精定位状态；final refiner status.
        self.create_subscription(String, 'autocharge/request', self.autocharge_request_cb, 10)  # 低电压回充请求；low-voltage autocharge request.
        self.parking_client = ActionClient(self, Parking, 'parking_action')  # 视觉泊车 action；vision parking action.
        self.camera_sub = self.create_subscription(
            Image, self.camera_image_topic, self.camera_image_cb, qos_profile_sensor_data)  # aruco 相机在线判定；aruco camera liveness.

        self.waypoints = self.load_waypoints(self.waypoints_file)
        self.region_to_waypoint = self.load_region_to_waypoint(self.waypoints_file)
        self.bridge_feedback = None
        self.refiner_status = None
        self.refiner_seq = 0
        self.refiner_history = []
        self.arm = None
        self.io_client = None
        self.bridge_connected_logged = False
        self.last_bridge_wait_log = 0.0
        self.autocharge_requested = False
        self.autocharge_level = 'normal'
        self.autocharge_active = False
        self.last_autocharge_wait_log = 0.0
        self.delivery_committed = False
        self.resolved_region = ''        # 取货 QR 解析出的目标分区；Target region resolved from pickup QR.

        self.last_camera_image_time = 0.0  # 最近一帧相机数据时间；Last camera frame time.
        self.arm_ok = False              # 机械臂可用（已读到角度）；Arm usable (angles read).
        self.qr_camera_ok = False        # 二维码相机可打开；QR camera openable.

        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints from {self.waypoints_file}')
        self.warn_if_pickup_is_destination()
        threading.Thread(target=self.startup_and_run, daemon=True).start()
        self.get_logger().info('Logistics node started. Checking subsystems before departure.')

    def resolve_arm_port(self):
        controller_device = '/dev/myagvplus_controller'
        controller = Path(controller_device).resolve().name
        port = '/dev/ttyACM1' if controller == 'ttyACM0' else '/dev/ttyACM0'
        self.get_logger().info(f'Arm serial port {port} ({controller_device} -> {controller}).')
        return port

    def camera_image_cb(self, msg):
        self.last_camera_image_time = time.monotonic()

    def camera_fresh(self):
        return (time.monotonic() - self.last_camera_image_time) <= self.camera_fresh_seconds

    def check_arm(self):
        if self.arm_ok:
            return True, 'ok'
        try:
            arm = self.get_arm()
        except Exception as error:
            return False, f'serial open failed: {self.arm_port} ({error})'
        ok, detail = arm.probe_arm()
        if ok:
            try:
                arm.move_angles(arm.angle_table['move_init'], 50)
                self.get_logger().info('Arm moved to move_init pose.')
            except Exception as error:
                return False, f'move to move_init failed: {error}'
        self.arm_ok = ok
        return ok, detail

    def check_qr_camera(self):
        if self.qr_camera_ok:
            return True, 'ok'
        scanner = None
        try:
            from .QRCodeScanner import QRCodeScanner
            scanner = QRCodeScanner(video_device=self.qr_camera, show_window=False)
            ok = (scanner.cap is not None and scanner.cap.isOpened()
                  and bool(scanner.cap.grab()))
        except Exception as error:
            return False, f'open failed: {self.qr_camera} ({error})'
        finally:
            if scanner is not None:
                scanner.release_resources()
        if not ok:
            return False, f'no frame: {self.qr_camera}'
        self.qr_camera_ok = True
        return True, 'ok'

    def startup_and_run(self):
        if not self.wait_until_ready():
            return
        self.get_logger().info(
            f'{GREEN}All subsystems ready. Logistics mission starting.{RESET}')
        if self.camera_sub is not None:
            self.destroy_subscription(self.camera_sub)
            self.camera_sub = None
        self.mission_loop()

    def wait_until_ready(self):
        last_log = 0.0
        while rclpy.ok():
            missing = []
            ok, detail = self.check_arm()
            if not ok:
                missing.append(f'arm({detail})')
            ok, detail = self.check_qr_camera()
            if not ok:
                missing.append(f'qr_camera({detail})')
            if self.goal_pose_pub.get_subscription_count() == 0:
                missing.append('route_bridge(navigation not up; start navigation2_active.launch.py)')
            if not self.parking_client.server_is_ready():
                missing.append(
                    'parking(action server /parking_action not ready; '
                    'check [parking-*] for "Parking Action Server Ready")')
            if not self.camera_fresh():
                missing.append(f'aruco_camera(no data on {self.camera_image_topic})')
            if not missing:
                return True
            now = time.monotonic()
            if now - last_log >= 1.0:
                last_log = now
                self.get_logger().error(
                    f'{RED}Waiting to depart. Not ready: ' + '; '.join(missing) + RESET)
            time.sleep(0.2)
        return False

    def mission_loop(self):
        while rclpy.ok():
            if self.autocharge_requested and not self.autocharge_active:
                if not self.run_autocharge():
                    time.sleep(1.0)
                continue
            if self.run_once():
                self.get_logger().info('Mission cycle completed. Continue next package.')
            else:
                self.get_logger().warn('Mission cycle failed. Retry while node remains running.')
                time.sleep(1.0)
        self.get_logger().info('Logistics mission loop stopped.')

    def run_autocharge(self):
        if self.autocharge_waypoint not in self.waypoints:
            self.get_logger().error(f'Autocharge waypoint {self.autocharge_waypoint} does not exist.')
            return False

        self.autocharge_active = True
        try:
            self.get_logger().warn(
                f'Autocharge requested: level={self.autocharge_level}, '
                f'waypoint={self.autocharge_waypoint}.')
            if not self.navigate_to_waypoint(self.autocharge_waypoint):
                self.get_logger().error('Autocharge navigation failed.')
                return False

            self.stop_robot()
            self.notify_autocharge_ready()
            self.get_logger().warn(
                'Autocharge waypoint reached and final pose refinement finished. '
                'USB-CAN docking trigger sent; waiting for voltage recovery.')
            self.wait_autocharge_recovery()
            return True
        finally:
            self.autocharge_active = False

    def notify_autocharge_ready(self):
        message = f'ready:{self.autocharge_level}:{self.autocharge_waypoint}'
        deadline = time.monotonic() + 2.0
        while (
            rclpy.ok()
            and self.autocharge_ready_pub.get_subscription_count() == 0
            and time.monotonic() < deadline
        ):
            self.stop_robot()
            time.sleep(0.1)

        for _ in range(10):
            self.autocharge_ready_pub.publish(String(data=message))
            self.stop_robot()
            time.sleep(0.1)

    def wait_autocharge_recovery(self):
        while rclpy.ok() and self.autocharge_requested:
            now = time.monotonic()
            if now - self.last_autocharge_wait_log >= 5.0:
                self.last_autocharge_wait_log = now
                self.get_logger().warn(f'Waiting for autocharge recovery: level={self.autocharge_level}.')
            self.stop_robot()
            time.sleep(1.0)
        self.get_logger().info('Autocharge request cleared. Resume logistics mission.')

    def stop_for_autocharge_if_needed(self):
        if not self.should_interrupt_for_autocharge():
            return False
        self.get_logger().error('Low battery autocharge requested. Stop current mission cycle.')
        self.stop_robot()
        return True

    def should_interrupt_for_autocharge(self):
        return self.autocharge_requested and not self.autocharge_active and not self.delivery_committed

    def cancel_route_bridge(self):
        self.route_cancel_pub.publish(Bool(data=True))
        self.cancel_final_pose_refiner()
        self.stop_robot()
        self.get_logger().error('Low battery requested. Cancel current route_bridge task.')

    def cancel_final_pose_refiner(self):
        if not self.refiner_param_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn('Final pose refiner parameter service is unavailable; skip refine cancel.')
            return

        parameter = ParameterMsg()
        parameter.name = 'final_pose_refiner_cancel'
        parameter.value = ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=True)
        request = SetParameters.Request()
        request.parameters = [parameter]
        future = self.refiner_param_client.call_async(request)
        if not self.wait_future(future, 0.5):
            self.get_logger().warn('Timed out canceling final pose refiner.')

    def run_once(self):
        self.delivery_committed = False
        self.get_logger().info(f'Mission step 1/4: navigate to pickup waypoint {self.pickup_waypoint}.')
        if not self.navigate_to_waypoint(self.pickup_waypoint):
            self.get_logger().error('Mission stopped: failed to reach pickup waypoint.')
            return False
        if self.stop_for_autocharge_if_needed():
            return False
        if not self.run_parking(self.pickup_waypoint, 'pickup'):
            self.get_logger().error('Mission stopped: pickup parking failed.')
            return False
        if self.stop_for_autocharge_if_needed():
            return False

        self.get_logger().info('Mission step 2/4: pick package and scan package QR with arm.')
        self.delivery_committed = True
        text = self.pick_package()
        if text is None:
            self.delivery_committed = False
            self.get_logger().error('Mission stopped: arm pick or package QR recognition failed.')
            return False
        self.retreat_from_shelf('pickup')
        if self.stop_for_autocharge_if_needed():
            return False

        waypoint = self.resolve_waypoint(text)
        if waypoint is None:
            self.get_logger().error('Mission stopped: QR text cannot be mapped to a waypoint.')
            return False
        if waypoint == self.pickup_waypoint:
            self.get_logger().warn(f'Destination waypoint is the same as pickup waypoint {self.pickup_waypoint}.')

        self.get_logger().info('Mission step 3/4: navigate to destination waypoint.')
        if not self.navigate_to_waypoint(waypoint):
            self.get_logger().error('Mission navigation failed.')
            return False
        if self.stop_for_autocharge_if_needed():
            return False
        if not self.run_parking(waypoint, 'destination', self.resolved_region):
            self.get_logger().error('Mission stopped: destination parking failed.')
            return False
        if self.stop_for_autocharge_if_needed():
            return False

        self.get_logger().info('Mission step 4/4: place package with arm.')
        if not self.place_package():
            self.get_logger().error('Mission stopped: arm place failed.')
            return False
        self.delivery_committed = False
        self.retreat_from_shelf('destination')
        self.get_logger().info('Mission navigation succeeded.')
        return True

    def navigate_to_waypoint(self, waypoint):
        if waypoint not in self.waypoints:
            self.get_logger().warn(f'Waypoint {waypoint} does not exist.')
            return False
        if not self.wait_for_bridge():
            return False

        pose = self.make_pose(waypoint)
        start_seq = self.refiner_seq
        self.bridge_feedback = None
        self.goal_pose_pub.publish(pose)
        self.get_logger().info(
            f'Sent route_bridge goal for waypoint {waypoint}: '
            f'x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}, '
            f'z={pose.pose.orientation.z:.3f}, w={pose.pose.orientation.w:.3f}')
        return self.wait_for_bridge_result() and self.wait_for_refiner(start_seq)

    def wait_for_bridge(self):
        self.get_logger().info('Waiting for route_bridge subscriber on /route_bridge/goal_pose.')
        while rclpy.ok():
            if self.should_interrupt_for_autocharge():
                self.get_logger().warn('Low battery requested while waiting for route_bridge. Switch to autocharge.')
                return False
            if self.goal_pose_pub.get_subscription_count() > 0:
                if not self.bridge_connected_logged:
                    self.bridge_connected_logged = True
                    self.get_logger().info(
                        f'{GREEN}Logistics route mission initialized: route_bridge connected.{RESET}')
                return True
            self.throttled_bridge_wait_log()
            time.sleep(0.1)
        return False

    def wait_for_bridge_result(self):
        deadline = time.monotonic() + 600.0  # route_bridge 导航结果最长等待时间，单位秒；Max wait for route_bridge result, in seconds.
        while rclpy.ok() and time.monotonic() < deadline:
            if self.should_interrupt_for_autocharge():
                self.cancel_route_bridge()
                return False
            if self.bridge_feedback in self.bridge_final:
                return self.bridge_feedback == 'route_bridge_succeeded'
            time.sleep(0.1)
        self.get_logger().warn('No final route_bridge feedback within 600.0 s.')
        return False

    def wait_for_refiner(self, start_seq):
        deadline = time.monotonic() + 30.0  # 最终姿态精修最长等待时间，单位秒；Max wait for final pose refinement, in seconds.
        saw_running = False
        sequence = start_seq
        while rclpy.ok() and time.monotonic() < deadline:
            if self.should_interrupt_for_autocharge():
                self.cancel_final_pose_refiner()
                self.stop_robot()
                return False
            updates = [(seq, status) for seq, status in self.refiner_history if seq > sequence]
            for sequence, status in updates:
                if status == 'running':
                    saw_running = True
                elif status == 'succeeded' and saw_running:
                    self.get_logger().info('Final pose refinement succeeded; parking is allowed to start.')
                    return True
                elif status in self.refiner_failed and (
                    saw_running or status in {'failed_tf', 'no_goal', 'handoff_distance_exceeded'}
                ):
                    self.get_logger().error(f'Final pose refinement failed: {status}.')
                    return False
            time.sleep(0.1)
        self.get_logger().warn('No final pose refinement success within 30.0 s.')
        return False

    def run_parking(self, waypoint, label, expect_region=''):
        self.get_logger().info(
            f'Start {label} parking at waypoint {waypoint}, marker_id=any visible marker'
            f"{f', expect_region={expect_region}' if expect_region else ''}.")
        if not self.parking_client.wait_for_server(timeout_sec=5.0):  # 泊车 action server 等待时间，单位秒；Parking action server wait timeout, in seconds.
            self.get_logger().error('Parking action server "parking_action" is not available.')
            return False

        goal = Parking.Goal()
        goal.marker_id = -1  # -1 表示使用任意可见 ArUco；-1 means use any visible ArUco marker.
        goal.expect_region = expect_region  # 放置时供 OCR 核对的目标分区，取货时为空；OCR target region for placement, empty for pickup.
        send_future = self.parking_client.send_goal_async(goal)
        if not self.wait_future(send_future, 5.0):
            self.get_logger().error('Timed out while sending parking goal.')
            return False

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Parking goal rejected.')
            return False

        result_future = goal_handle.get_result_async()
        if not self.wait_future(result_future, 180.0):  # 泊车动作结果最长等待时间，单位秒；Max wait for parking action result, in seconds.
            self.get_logger().error('Timed out while waiting for parking result.')
            return False

        result = result_future.result().result
        self.get_logger().info(f'Parking result: success={result.success}, message={result.message}')
        return bool(result.success)

    def pick_package(self):
        box_height = self.arm_fallback_height
        self.get_logger().info(
            f'Fallback pick package height: height={box_height:.1f}.')
        try:
            result = self.get_arm().pick(box_height)
        except Exception as error:
            self.get_logger().error(f'Arm pick failed: {error}')
            return None
        if result is None:
            return None
        return result

    def place_package(self):
        try:
            return bool(self.get_arm().place())
        except Exception as error:
            self.get_logger().error(f'Arm place failed: {error}')
            return False

    def retreat_from_shelf(self, label):
        if self.retreat_distance <= 0.0 or self.retreat_speed <= 0.0:
            return
        duration = self.retreat_distance / self.retreat_speed
        self.get_logger().info(
            f'Retreat after {label}: distance={self.retreat_distance:.3f} m, '
            f'speed={self.retreat_speed:.3f} m/s.')
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.twist.linear.x = -self.retreat_speed
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self.stop_robot()

    def stop_robot(self):
        twist = TwistStamped()
        twist.header.stamp = self.get_clock().now().to_msg()
        self.cmd_vel_pub.publish(twist)

    def get_arm(self):
        if self.arm is None:
            from .arm_controller import MechArm270Control
            from ros_client import AGVIOClient
            self.io_client = AGVIOClient()
            self.arm = MechArm270Control(
                port=self.arm_port,  # 机械臂串口；Arm serial port.
                baudrate=self.arm_baudrate,  # 机械臂串口波特率；Arm serial baudrate.
                qr_camera=self.qr_camera,  # 二维码相机设备；QR camera device.
                qr_timeout=self.qr_timeout,  # 二维码识别超时，单位秒；QR scan timeout, in seconds.
                qr_show_window=True,  # 显示二维码识别窗口；Show QR scan window.
                io_client=self.io_client)  # 底部协议吸泵控制；Bottom-protocol pump control.
        return self.arm

    def make_pose(self, waypoint):
        x, y, z, w = self.waypoints[waypoint]
        norm = math.hypot(z, w)
        if norm < 1e-6:
            raise RuntimeError(f'waypoint {waypoint} has invalid orientation: z={z}, w={w}')
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x, pose.pose.position.y = x, y
        pose.pose.orientation.z, pose.pose.orientation.w = z / norm, w / norm
        return pose

    def resolve_waypoint(self, text):
        region = next((region for city, region in self.city_to_region.items() if city in text), None)
        if region is None:
            region = next((region for region in self.region_to_waypoint if region in text), None)
        waypoint = self.region_to_waypoint.get(region)
        if waypoint in self.waypoints:
            self.resolved_region = region or ''  # 记录分区供放置时 OCR 核对；Keep region for OCR check at placement.
            self.get_logger().info(f'Resolved QR text "{text}" -> region "{region}" -> waypoint {waypoint}')
            return waypoint
        self.get_logger().warn(f'No valid waypoint mapping for QR text: {text}')
        return None

    def bridge_feedback_cb(self, msg):
        self.bridge_feedback = msg.data
        self.get_logger().info(f'route_bridge feedback: {msg.data}')

    def autocharge_request_cb(self, msg):
        level = msg.data.strip().lower()
        if level in {'normal', 'recovered', 'clear'}:
            if self.autocharge_requested:
                self.get_logger().info('Autocharge request cleared by monitor.')
            self.autocharge_requested = False
            self.autocharge_level = 'normal'
            return
        if level != 'warning':
            self.get_logger().warn(f'Ignore invalid autocharge request: {msg.data}')
            return

        if not self.autocharge_requested or self.autocharge_level != level:
            self.get_logger().warn(f'Autocharge request received: level={level}.')
        self.autocharge_requested = True
        self.autocharge_level = level

    def refiner_status_cb(self, msg):
        status = msg.data.strip()
        if status:
            self.refiner_status = status
            self.refiner_seq += 1
            self.refiner_history.append((self.refiner_seq, status))
            self.refiner_history = self.refiner_history[-32:]

    def wait_future(self, future, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if future.done():
                return True
            time.sleep(0.1)
        return future.done()

    def throttled_bridge_wait_log(self):
        now = time.monotonic()
        if now - self.last_bridge_wait_log >= 5.0:
            self.last_bridge_wait_log = now
            self.get_logger().warn(
                'Route bridge waiting for /route_bridge/goal_pose subscriber. '
                'Start navigation2_active.launch.py first.')

    def warn_if_pickup_is_destination(self):
        for region, waypoint in self.region_to_waypoint.items():
            if waypoint == self.pickup_waypoint:
                self.get_logger().warn(
                    f'Region {region} maps to pickup waypoint {self.pickup_waypoint}. '
                    'Update self.region_to_waypoint if the pickup point is not a destination.')

    def load_waypoints(self, waypoints_file):
        path = Path(waypoints_file)
        data = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
        waypoints = {}
        for key, value in (data.get('waypoints') or {}).items():
            name = str(key).strip().upper()
            waypoints[name] = [float(item) for item in value]
        return waypoints

    def load_region_to_waypoint(self, waypoints_file):
        mapping = dict(self.default_region_to_waypoint)
        data = yaml.safe_load(Path(waypoints_file).read_text(encoding='utf-8')) or {}
        loaded = data.get('region_to_waypoint') or {}
        if not loaded:
            self.get_logger().warn(
                'No region_to_waypoint field in waypoints.yaml. Using built-in defaults. '
                'Run scripts/map_regions.py to generate it.')
            return mapping
        for region, waypoint in loaded.items():
            mapping[str(region).strip()] = str(waypoint).strip().upper()
        self.get_logger().info(f'Loaded region mapping from {waypoints_file}: {mapping}')
        return mapping

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = LogisticsRouteMission()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f'{RED}[main] {error}{RESET}')
    finally:
        if node is not None:
            if node.io_client is not None:
                node.io_client.destroy_node()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
