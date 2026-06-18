#!/usr/bin/env python3

from collections import deque
import sys
import threading
import time

from geometry_msgs.msg import TwistStamped
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

try:
    from .wit_usb2can import SerialCANParser, mode_name
except ImportError:
    from smart_logistics_kit.wit_usb2can import SerialCANParser, mode_name


GREEN = '\033[32m'
YELLOW = '\033[33m'
RED = '\033[31m'
RESET = '\033[0m'


class AutochargeCoordinator(Node):
    def __init__(self):
        super().__init__('autocharge_coordinator')

        self.warning_voltage = 19.6  # 回充触发电压阈值，单位 V；Voltage threshold to request autocharge, in volts.
        self.recover_voltage = 24.0  # 回充恢复电压阈值，单位 V；Voltage threshold to clear autocharge request, in volts.
        self.stable_seconds = 2.0  # 电压等级稳定窗口，单位秒；Voltage level stability window, in seconds.
        self.log_interval = 5.0  # 普通节流日志间隔，单位秒；General throttled log interval, in seconds.
        self.voltage_stale_seconds = 30.0  # 电压话题失联判定时间，单位秒；Voltage topic stale timeout, in seconds.
        self.charge_current_threshold_ma = 200.0  # 充电电流判定阈值，单位 mA；Charging current threshold, in mA.

        self.voltage_samples = deque(maxlen=200)
        self.last_voltage = None
        self.last_voltage_time = 0.0
        self.stable_level = None
        self.stable_level_since = 0.0
        self.last_level = None
        self.last_wait_log = 0.0
        self.request_active = False
        self.request_level = 'normal'
        self.initialized = False

        self.shutdown_requested = False
        self.docking_active = False
        self.docking_done = False
        self.docking_thread = None
        self.docking_lock = threading.Lock()
        self.last_can_wait_log = 0.0
        self.last_can_frame_log = 0.0
        self.last_ready_ignore_log = 0.0
        self.last_return_to_charge_log = 0.0
        self.last_charging_log = 0.0

        self.request_pub = self.create_publisher(String, 'autocharge/request', 10)  # 回充请求输出；Autocharge request output.
        self.state_pub = self.create_publisher(String, 'autocharge/state', 10)  # 回充状态输出；Autocharge state output.
        self.cmd_vel_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)  # 最后一米速度控制；Final docking velocity command.
        self.create_subscription(Float32, '/voltage', self.voltage_cb, 10)  # 电池电压输入；Battery voltage input.
        self.create_subscription(String, 'autocharge/ready', self.autocharge_ready_cb, 10)  # P 点到达触发；P waypoint ready trigger.
        self.create_timer(1.0, self.timer_cb)

        self.get_logger().warn(
            f'Autocharge monitor waiting for stable voltage. topic=/voltage, '
            f'stable_seconds={self.stable_seconds:.1f}, '
            f'warning={self.warning_voltage:.1f} V, recover={self.recover_voltage:.1f} V')
        self.get_logger().info(
            'USB-CAN docking gate: ready_topic=autocharge/ready, '
            'port=/dev/ttyCH341USB1, cmd_vel=/cmd_vel')

    def voltage_cb(self, msg):
        now = time.monotonic()
        raw_voltage = float(msg.data)
        self.last_voltage = raw_voltage
        self.last_voltage_time = now
        self.voltage_samples.append((now, raw_voltage))
        self.drop_old_voltage_samples(now)

        if not self.initialized and raw_voltage < self.warning_voltage:
            self.initialized = True
            self.last_level = 'warning'
            self.request_active = True
            self.request_level = 'warning'
            self.docking_done = False
            self.request_pub.publish(String(data='warning'))
            self.publish_state(f'warning:{raw_voltage:.1f}')
            self.get_logger().warn(
                f'{YELLOW}Startup low voltage: voltage={raw_voltage:.1f} V < '
                f'{self.warning_voltage:.1f} V. Request autocharge immediately.{RESET}')
            self.throttled_return_to_charge_log(raw_voltage)
            return

        stable = self.get_stable_voltage(now)
        if stable is None:
            if self.request_active:
                self.request_pub.publish(String(data=self.request_level))
            state_level = self.request_level if self.request_active else 'sampling'
            self.publish_state(f'{state_level}:{raw_voltage:.1f}')
            return

        voltage, level = stable
        if not self.initialized:
            self.initialized = True
            self.get_logger().info(
                f'{GREEN}Autocharge monitor initialized: stable_voltage={voltage:.1f} V, '
                f'level={level}.{RESET}')

        if level != self.last_level:
            if level == 'warning':
                self.get_logger().warn(
                    f'{YELLOW}Battery warning: voltage={voltage:.1f} V < '
                    f'{self.warning_voltage:.1f} V.{RESET}')
            else:
                self.get_logger().info(f'{GREEN}Battery normal: voltage={voltage:.1f} V.{RESET}')
            self.last_level = level

        if voltage < self.warning_voltage:
            if not self.request_active:
                self.docking_done = False
            self.request_active = True
            self.request_level = level
            self.request_pub.publish(String(data=level))
        elif self.request_active and voltage >= self.recover_voltage:
            self.request_active = False
            self.request_level = 'normal'
            self.docking_done = False
            self.request_pub.publish(String(data='normal'))
            self.get_logger().info(
                f'{GREEN}Battery recovered: voltage={voltage:.1f} V >= '
                f'{self.recover_voltage:.1f} V.{RESET}')
        elif self.request_active:
            self.request_pub.publish(String(data=self.request_level))

        state_level = self.request_level if self.request_active else level
        self.publish_state(f'{state_level}:{voltage:.1f}')
        if self.request_active and not self.docking_done:
            self.throttled_return_to_charge_log(voltage)
        elif self.docking_done:
            self.throttled_charging_log(voltage)

    def autocharge_ready_cb(self, msg):
        token = msg.data.strip() or 'ready'
        if token.lower() in {'normal', 'clear', 'cancel'}:
            return
        if not self.request_active:
            self.throttled_ready_ignore_log(
                f'Ignore autocharge ready "{token}": no active low-voltage request.')
            self.publish_state('ready_ignored:no_request')
            return

        with self.docking_lock:
            if self.docking_active or self.docking_done:
                return
            self.docking_active = True

        self.publish_state(f'docking_start:{token}')
        self.get_logger().warn(
            f'{YELLOW}Autocharge ready received: {token}. Starting USB-CAN final docking.{RESET}')
        self.docking_thread = threading.Thread(target=self.run_can_docking, args=(token,), daemon=True)
        self.docking_thread.start()

    def run_can_docking(self, token):
        parser = None
        result = 'stopped'
        frame_count = 0
        deadline = time.monotonic() + 180.0  # USB-CAN 最后一米最长运行时间，单位秒；USB-CAN final docking max runtime, in seconds.

        try:
            parser = SerialCANParser('/dev/ttyCH341USB1', 9600, 1.0, False)  # USB-CAN 串口、波特率、读超时、调试开关；USB-CAN port, baudrate, timeout, debug flag.
            parser.open_serial()
            parser.send_at_commands(['AT+CG', 'AT+AT'])
            self.publish_state('can_started')
            self.get_logger().info(f'{GREEN}USB-CAN serial control started.{RESET}')

            while rclpy.ok() and not self.shutdown_requested:
                if not self.request_active:
                    result = 'request_cleared'
                    break
                if time.monotonic() >= deadline:
                    result = 'timeout'
                    self.get_logger().error(f'{RED}USB-CAN docking timed out after 180.0 s.{RESET}')
                    break

                frame = parser.read_frame(2.0)  # 0x182 引导帧等待超时，单位秒；0x182 guide frame wait timeout, in seconds.
                if frame is None:
                    self.publish_cmd_vel()
                    self.throttled_can_wait_log()
                    self.publish_state('can_waiting')
                    continue

                frame_count += 1
                result = self.handle_can_frame(frame, frame_count)
                if result is not None:
                    break
                result = 'running'
        except Exception as error:
            result = 'error'
            self.get_logger().error(f'{RED}USB-CAN docking error: {error}{RESET}')
        finally:
            self.publish_cmd_vel()
            if parser is not None:
                parser.close_serial()
            with self.docking_lock:
                self.docking_active = False
                if result == 'charging_zone':
                    self.docking_done = True
                    self.throttled_charging_log(self.last_voltage)
            self.publish_state(f'docking_finished:{result}')
            self.get_logger().warn(
                f'{YELLOW}USB-CAN docking stopped: result={result}, '
                f'frames={frame_count}, trigger={token}.{RESET}')

    def handle_can_frame(self, frame, frame_count):
        bits = frame['infrared_bits']
        mode = frame['mode']
        self.throttled_can_frame_log(frame, frame_count)
        self.publish_state(
            f"can:{mode_name(mode)}:{frame['x_speed']:.3f}:"
            f"{frame['z_speed']:.3f}:{frame['actual_current']:.1f}")

        if frame['actual_current'] >= self.charge_current_threshold_ma:
            self.publish_cmd_vel()
            self.get_logger().info(
                f'{GREEN}Charging current detected: {frame["actual_current"]:.1f} mA >= '
                f'{self.charge_current_threshold_ma:.1f} mA. Stop final docking.{RESET}')
            return 'charging_zone'

        if bits[7] != 0:
            self.publish_cmd_vel()
            self.get_logger().warn(
                f'{YELLOW}Charging flag / obstacle stop. Stop final docking without charging success.{RESET}')
            return 'charging_flag_stop'

        if mode == 0x01:
            self.publish_cmd_vel(frame['x_speed'], frame['z_speed'])
            return None

        if mode == 0xBB:
            self.publish_cmd_vel()
            return None

        if mode == 0xAA:
            self.publish_cmd_vel()
            self.get_logger().info(f'{GREEN}Charging zone detected. Stop final docking.{RESET}')
            return 'charging_zone'

        if mode == 0xCF:
            self.publish_cmd_vel()
            self.get_logger().error(f'{RED}Emergency stop mode detected.{RESET}')
            return 'emergency_stop'

        self.publish_cmd_vel()
        self.get_logger().warn(f'Unknown USB-CAN mode 0x{mode:02X}; publish stop and keep observing.')
        return None

    def publish_cmd_vel(self, x_speed=0.0, z_speed=0.0):  # x/z 速度来自回充板 0x182 引导帧；x/z speeds come from the charger 0x182 guide frame.
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(x_speed)
        msg.twist.angular.z = float(z_speed)
        self.cmd_vel_pub.publish(msg)

    def publish_state(self, text):
        self.state_pub.publish(String(data=text))

    def get_stable_voltage(self, now):
        if len(self.voltage_samples) < 2:
            return None

        _, latest_voltage = self.voltage_samples[-1]
        level = self.get_level(latest_voltage)
        if level != self.stable_level:
            self.stable_level = level
            self.stable_level_since = now
            return None

        duration = now - self.stable_level_since
        if duration < self.stable_seconds:
            self.throttled_voltage_wait_log(
                f'Waiting for stable voltage window: {duration:.1f}/{self.stable_seconds:.1f} s.')
            return None

        window = [
            voltage
            for stamp, voltage in self.voltage_samples
            if stamp >= self.stable_level_since and self.get_level(voltage) == level
        ]
        if not window:
            return latest_voltage, level

        voltage = sum(window) / len(window)
        return voltage, level

    def drop_old_voltage_samples(self, now):
        max_age = max(self.stable_seconds + 2.0, self.stable_seconds * 2.0)
        while self.voltage_samples and now - self.voltage_samples[0][0] > max_age:
            self.voltage_samples.popleft()

    def timer_cb(self):
        now = time.monotonic()
        if self.last_voltage is None:
            if now - self.last_wait_log >= self.log_interval:
                self.last_wait_log = now
                self.get_logger().warn('Waiting for voltage topic: /voltage')
                self.publish_state('waiting_voltage_topic')
            return

        age = now - self.last_voltage_time
        if age >= self.voltage_stale_seconds and now - self.last_wait_log >= self.log_interval:
            self.last_wait_log = now
            self.get_logger().warn(f'Voltage topic stale: last /voltage sample was {age:.1f} s ago.')
            self.publish_state(f'voltage_stale:{age:.1f}')

    def get_level(self, voltage):
        return 'warning' if voltage < self.warning_voltage else 'normal'

    def throttled_voltage_wait_log(self, message):
        now = time.monotonic()
        if now - self.last_wait_log >= self.log_interval:
            self.last_wait_log = now
            self.get_logger().warn(message)

    def throttled_ready_ignore_log(self, message):
        now = time.monotonic()
        if now - self.last_ready_ignore_log >= self.log_interval:
            self.last_ready_ignore_log = now
            self.get_logger().warn(message)

    def throttled_can_wait_log(self):
        now = time.monotonic()
        if now - self.last_can_wait_log >= self.log_interval:
            self.last_can_wait_log = now
            self.get_logger().warn('Waiting for valid USB-CAN 0x182 frame.')

    def throttled_can_frame_log(self, frame, frame_count):
        now = time.monotonic()
        if now - self.last_can_frame_log < 1.0:
            return
        self.last_can_frame_log = now
        bits = frame['infrared_bits']
        self.get_logger().info(
            f"CAN frame={frame_count}, mode={mode_name(frame['mode'])}, "
            f"x={frame['x_speed']:.3f}, z={frame['z_speed']:.3f}, "
            f"L_A={bits[2]}, L_B={bits[3]}, R_B={bits[4]}, R_A={bits[5]}, "
            f"infrared_flag={bits[6]}, charging_flag={bits[7]}, "
            f"current={frame['actual_current']:.1f} mA")

    def throttled_return_to_charge_log(self, voltage):
        now = time.monotonic()
        if now - self.last_return_to_charge_log < 1.0:
            return
        self.last_return_to_charge_log = now
        self.get_logger().warn(
            f'{YELLOW}电压为 {voltage:.1f}V，过低，需要充电，正在返回 P 点充电。{RESET}')

    def throttled_charging_log(self, voltage):
        now = time.monotonic()
        if now - self.last_charging_log < 5.0:
            return
        self.last_charging_log = now
        voltage_text = '--' if voltage is None else f'{voltage:.1f}'
        self.get_logger().info(f'{GREEN}电压为 {voltage_text}V，正在充电...{RESET}')

    def stop(self):
        self.shutdown_requested = True
        self.publish_cmd_vel()
        thread = self.docking_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=2.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AutochargeCoordinator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f'{RED}[autocharge_coordinator] {error}{RESET}')
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
