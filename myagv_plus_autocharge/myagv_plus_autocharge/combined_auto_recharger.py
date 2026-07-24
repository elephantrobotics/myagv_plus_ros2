#!/usr/bin/env python3
import os
import sys
import json
import math
import time
import select
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, TwistStamped
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import Buffer, TransformListener
import yaml

from ament_index_python.packages import get_package_share_directory
from .wit_usb2can import SerialCANParser

if os.name != 'nt':
    import termios
    import tty

_pkg_dir = get_package_share_directory('myagv_plus_autocharge')
CONFIG_DIR = os.path.abspath(os.path.join(
    _pkg_dir, '..', '..', '..', '..', 'src', 'myagv_plus_autocharge', 'config'))
JSON_FILE = os.path.join(CONFIG_DIR, 'charger_position.json')
YAML_FILE = os.path.join(CONFIG_DIR, 'nav_goal_params.yaml')

GREEN = '\033[1;32m'
RED = '\033[1;31m'
YELLOW = '\033[1;33m'
BLUE = '\033[1;34m'
RESET = '\033[0m'

settings = None
if os.name != 'nt' and sys.stdin.isatty():
    settings = list(termios.tcgetattr(sys.stdin))


def get_key():
    if os.name == 'nt':
        import msvcrt
        return msvcrt.getch().decode('utf-8')
    if sys.stdin.isatty():
        tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = ''
    if rlist:
        key = sys.stdin.read(1)
    if sys.stdin.isatty() and settings:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def safe_print(text):
    if sys.stdin.isatty() and settings:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    print(text)


class CombinedAutoRecharger(Node):
    def __init__(self):
        super().__init__('combined_auto_recharger')

        self.navigation_active = False
        self.serial_control_active = False
        self.parser = None
        self.charge_current_threshold = 200.0

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.load_charger_position()
        self.load_nav_params()

        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 1)
        self.marker_pub = self.create_publisher(MarkerArray, '/goal_marker', 10)

        self.charger_update_sub = self.create_subscription(
            PoseStamped, '/charger_position_update',
            self.charger_position_update_callback, 10)

        self.marker_timer = self.create_timer(2.0, self.publish_marker)
        self.publish_marker()

    def make_twist_stamped(self, linear_x=0.0, angular_z=0.0):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = linear_x
        msg.twist.angular.z = angular_z
        return msg

    def charger_position_update_callback(self, msg):
        self.charger = {
            'p_x': msg.pose.position.x,
            'p_y': msg.pose.position.y,
            'orien_z': msg.pose.orientation.z,
            'orien_w': msg.pose.orientation.w
        }
        with open(JSON_FILE, 'w') as f:
            json.dump(self.charger, f, indent=2)
        safe_print(f'{GREEN}Charger position updated via RViz: '
                   f'x={self.charger["p_x"]:.4f}, y={self.charger["p_y"]:.4f}{RESET}')
        self.publish_marker()

    def load_charger_position(self):
        with open(JSON_FILE, 'r') as f:
            self.charger = json.load(f)
        safe_print(f'Charger position: x={self.charger["p_x"]:.3f}, y={self.charger["p_y"]:.3f}')

    def load_nav_params(self):
        with open(YAML_FILE, 'r') as f:
            params = yaml.safe_load(f)
        self.forward_distance = float(params['forward_distance'])
        self.yaw_offset_deg = float(params['yaw_offset_deg'])
        safe_print(f'Nav params: forward_distance={self.forward_distance}, '
                   f'yaw_offset={self.yaw_offset_deg}°')

    def record_charger_pose(self):
        safe_print(f'{BLUE}Sampling pose (1s, keep still)...{RESET}')
        xs, ys, zs, ws = [], [], [], []
        dt = 1.0 / 20
        start = time.time()
        while time.time() - start < 1.0:
            try:
                trans = self.tf_buffer.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time())
                t = trans.transform.translation
                r = trans.transform.rotation
                xs.append(t.x)
                ys.append(t.y)
                zs.append(r.z)
                ws.append(r.w)
            except Exception:
                pass
            time.sleep(dt)

        if not xs:
            safe_print(f'{RED}Failed to sample pose. Is AMCL running?{RESET}')
            return

        xs.sort(); ys.sort(); zs.sort(); ws.sort()
        n = len(xs)
        mid = n // 2
        if n % 2 == 1:
            px, py, oz, ow = xs[mid], ys[mid], zs[mid], ws[mid]
        else:
            px = (xs[mid - 1] + xs[mid]) / 2
            py = (ys[mid - 1] + ys[mid]) / 2
            oz = (zs[mid - 1] + zs[mid]) / 2
            ow = (ws[mid - 1] + ws[mid]) / 2

        self.charger = {
            'p_x': round(px, 6),
            'p_y': round(py, 6),
            'orien_z': round(oz, 6),
            'orien_w': round(ow, 6)
        }

        with open(JSON_FILE, 'w') as f:
            json.dump(self.charger, f, indent=2)

        safe_print(f'{GREEN}Charger pose saved: x={px:.4f}, y={py:.4f}, oz={oz:.4f}, ow={ow:.4f}{RESET}')
        self.publish_marker()

    def publish_marker(self):
        marker_array = MarkerArray()

        arrow = Marker()
        arrow.id = 0
        arrow.header.frame_id = 'map'
        arrow.header.stamp = self.get_clock().now().to_msg()
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.scale.x = 0.5
        arrow.scale.y = 0.05
        arrow.scale.z = 0.05
        arrow.pose.position.x = self.charger['p_x']
        arrow.pose.position.y = self.charger['p_y']
        arrow.pose.position.z = 0.1
        arrow.pose.orientation.z = self.charger['orien_z']
        arrow.pose.orientation.w = self.charger['orien_w']
        arrow.color.r = 1.0
        arrow.color.g = 0.0
        arrow.color.b = 0.0
        arrow.color.a = 1.0
        marker_array.markers.append(arrow)

        text = Marker()
        text.id = 1
        text.header.frame_id = 'map'
        text.header.stamp = self.get_clock().now().to_msg()
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.scale.z = 0.3
        text.pose.position.x = self.charger['p_x']
        text.pose.position.y = self.charger['p_y']
        text.pose.position.z = 0.5
        text.color.r = 1.0
        text.color.g = 0.0
        text.color.b = 0.0
        text.color.a = 1.0
        text.text = 'Charger'
        marker_array.markers.append(text)

        self.marker_pub.publish(marker_array)

    def execute_navigation(self):
        if self.navigation_active:
            safe_print(f'{YELLOW}Navigation already in progress{RESET}')
            return

        self.navigation_active = True
        try:
            safe_print(f'{BLUE}Starting navigation to charger...{RESET}')

            px = self.charger['p_x']
            py = self.charger['p_y']
            oz = self.charger['orien_z']
            ow = self.charger['orien_w']
            yaw = 2 * math.atan2(oz, ow)

            x_offset = self.forward_distance * math.cos(yaw)
            y_offset = self.forward_distance * math.sin(yaw)
            goal_x = px + x_offset
            goal_y = py + y_offset

            goal_yaw = yaw - math.radians(self.yaw_offset_deg)
            goal_qz = math.sin(goal_yaw / 2)
            goal_qw = math.cos(goal_yaw / 2)

            safe_print(f'Nav goal: x={goal_x:.3f}, y={goal_y:.3f}, yaw={math.degrees(goal_yaw):.1f}°')

            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = goal_x
            goal_pose.pose.position.y = goal_y
            goal_pose.pose.orientation.z = goal_qz
            goal_pose.pose.orientation.w = goal_qw

            if not self.nav_client.wait_for_server(timeout_sec=5.0):
                safe_print(f'{RED}NavigateToPose action server not available{RESET}')
                return

            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal_pose
            send_goal_future = self.nav_client.send_goal_async(goal_msg)
            while not send_goal_future.done():
                time.sleep(0.1)
            goal_handle = send_goal_future.result()
            if not goal_handle or not goal_handle.accepted:
                safe_print(f'{RED}Navigation goal rejected{RESET}')
                return

            result_future = goal_handle.get_result_async()
            while not result_future.done():
                time.sleep(0.5)

            status = result_future.result().status
            if status == 4:
                safe_print(f'{GREEN}Navigation succeeded! Starting serial docking...{RESET}')
                self.start_serial_docking()
            else:
                safe_print(f'{RED}Navigation failed (status={status}){RESET}')
        except Exception as e:
            safe_print(f'{RED}Navigation error: {e}{RESET}')
        finally:
            self.navigation_active = False

    def start_serial_docking(self):
        self.serial_control_active = True
        try:
            self.parser = SerialCANParser('/dev/ttyCH341USB1', 9600, 1, debug=False)
            self.parser.open_serial()
            self.parser.send_at_commands(["AT+CG", "AT+AT"])

            safe_print(f'{BLUE}Serial docking started, reading charger frames...{RESET}')
            last_mode = None
            pressure_total_start = None
            docking_phase_printed = 0

            while self.serial_control_active:
                frame = self.parser.read_frame(2.0)
                if frame is None:
                    self.cmd_vel_pub.publish(self.make_twist_stamped())
                    continue

                mode = frame['mode']
                bits = frame['infrared_bits']

                if frame['actual_current'] >= self.charge_current_threshold:
                    self.cmd_vel_pub.publish(self.make_twist_stamped())
                    safe_print(f'{GREEN}Charging current detected: '
                               f'{frame["actual_current"]:.1f} mA - docking complete{RESET}')
                    break

                if bits[7] != 0:
                    self.cmd_vel_pub.publish(self.make_twist_stamped())
                    safe_print(f'{YELLOW}Charging flag - stopped{RESET}')
                    break

                if mode == 0x01:
                    self.cmd_vel_pub.publish(
                        self.make_twist_stamped(frame['x_speed'], frame['z_speed']))
                    if last_mode != 0x01:
                        phase = 2 if pressure_total_start and time.time() - pressure_total_start >= 5.0 else 1
                        if docking_phase_printed < phase:
                            suffix = ' (retrying)' if phase == 2 else ''
                            safe_print(f'Docking in progress...{suffix}')
                            docking_phase_printed = phase
                        last_mode = 0x01
                elif mode == 0xBB:
                    now = time.time()
                    if pressure_total_start is None:
                        pressure_total_start = now
                    if last_mode != 0xBB:
                        last_mode = 0xBB
                        pressure_start = now
                    total_elapsed = now - pressure_total_start
                    if total_elapsed >= 10.0:
                        self.cmd_vel_pub.publish(self.make_twist_stamped())
                        safe_print(f'{RED}Pressure retry timeout - stopped{RESET}')
                        break
                    pull_duration = 1.0 if total_elapsed >= 5.0 else 0.5
                    elapsed = now - pressure_start
                    if elapsed < pull_duration:
                        self.cmd_vel_pub.publish(
                            self.make_twist_stamped(linear_x=0.05))
                    else:
                        self.cmd_vel_pub.publish(self.make_twist_stamped())
                        last_mode = None
                elif mode == 0xAA:
                    self.cmd_vel_pub.publish(self.make_twist_stamped())
                    safe_print(f'{GREEN}Charging zone reached - docking complete{RESET}')
                    break
                elif mode == 0xCF:
                    self.cmd_vel_pub.publish(self.make_twist_stamped())
                    safe_print(f'{RED}Emergency stop{RESET}')
                    break

        except Exception as e:
            safe_print(f'{RED}Serial docking error: {e}{RESET}')
            self.cmd_vel_pub.publish(self.make_twist_stamped())
        finally:
            if self.parser:
                self.parser.close_serial()
            self.serial_control_active = False
            safe_print('Serial docking ended.')


def main(args=None):
    rclpy.init(args=args)
    node = CombinedAutoRecharger()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    safe_print(f'{GREEN}Enter: record charger pose | p: start auto recharge{RESET}')

    try:
        while True:
            key = get_key()
            if key == '\r' or key == '\n':
                node.record_charger_pose()
            elif key.lower() == 'p':
                if not node.navigation_active:
                    nav_thread = threading.Thread(
                        target=node.execute_navigation, daemon=True)
                    nav_thread.start()
            elif key == '\x03':
                break
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_vel_pub.publish(node.make_twist_stamped())
        node.destroy_node()
        rclpy.shutdown()
        safe_print('Exited.')


if __name__ == '__main__':
    main()
