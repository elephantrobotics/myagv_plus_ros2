#!/usr/bin/env python3
"""
手柄控制节点
- 接收手柄输入
- 发布速度控制指令
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from myagv_plus_joystick_control.joystick import InputJoystick, Hotkey
from myagv_plus_joystick_control.mecharm_controller import MechArm270Controller
from ros_client import AGVIOClient

import queue
import threading
import time
import os

DEADZONE = 15  # 左摇杆死区


class JoystickController(Node):
    def __init__(self):
        super().__init__('joystick_controller')

        # 速度参数
        self.max_linear_speed = 0.25
        self.max_angular_speed = 1.0

        # 当前速度
        self.linear_speed_x = 0.0
        self.linear_speed_y = 0.0
        self.angular_speed = 0.0

        self._vertical_active = False
        self._last_vertical_stop = 0.0

        self._horizontal_active = False
        self._last_horizontal_stop = 0.0

        self._gripper_active = False
        self._last_gripper_stop = 0.0

        self._left_x_active = 0   
        self._last_left_x_stop = 0.0

        self._left_y_active = 0
        self._last_left_y_stop = 0.0

        self._arm_cmd_queue = queue.Queue(maxsize=1)
        self._arm_running = True

        self._joystick_connected = False
        self._joystick_thread = None
        self._speed_timer = None
        self._joystick_warn_logged = False

        self.arm_controller = None
        chassis_port = self._get_chassis_serial_port()
        self.get_logger().info(f"底盘串口占用: {chassis_port}")

        self.agv_io = None
        try:
            self.agv_io = AGVIOClient()  # 自动等待 /query_device 服务
            self.get_logger().info("AGVIOClient initialized successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize AGVIOClient: {e}")
            self.agv_io = None

        self.arm_controller = None
        chassis_port = self._get_chassis_serial_port()
        self.get_logger().info(f"底盘串口占用: {chassis_port}")
        
        candidate_ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2']
        if chassis_port and chassis_port in candidate_ports:
            candidate_ports.remove(chassis_port)
            self.get_logger().info(f"已排除底盘端口，尝试机械臂端口: {candidate_ports}")

        for port in candidate_ports:
            try:
                self.arm_controller = MechArm270Controller(
                    port=port, 
                    agv_io_client=self.agv_io,  # 关键：传入AGV IO客户端
                    debug=False
                )
                self.get_logger().info(f"机械臂控制器初始化成功 | 端口: {port}")
                break
            except Exception:
                continue

        if self.arm_controller is None:
            from myagv_plus_joystick_control.mecharm_controller import UndefinedController
            self.arm_controller = UndefinedController()
            self.get_logger().warn("未检测到机械臂")

        self._arm_worker_thread = threading.Thread(target=self._arm_worker_loop)
        self._arm_worker_thread.daemon = True
        self._arm_worker_thread.start()

        self.cmd_vel_publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)

        self.joystick = InputJoystick(raw_mapping=False)
        self.joystick.inject_caller(self)
        self.register_joystick_events()

        self._try_connect_joystick()
        if not self._joystick_connected:
            self._joystick_check_timer = self.create_timer(2.0, self._try_connect_joystick)
            self.get_logger().warn("未检测到手柄,重试连接...")

    def _get_chassis_serial_port(self) -> str:
        """获取底盘控制器占用的实际串口路径
        
        通过读取 /dev/myagvplus_controller 符号链接目标，
        确定底盘占用了哪个 ttyACM 端口，从而排除该端口用于机械臂连接。
        
        Returns:
            str: 底盘占用的串口路径，如 '/dev/ttyACM1'；未找到则返回空字符串
        """
        chassis_symlink = '/dev/myagvplus_controller'
        
        if not os.path.exists(chassis_symlink):
            self.get_logger().warn(f"底盘符号链接不存在: {chassis_symlink}")
            return ""
        
        try:
            if os.path.islink(chassis_symlink):
                target = os.readlink(chassis_symlink)
                if target.startswith('/'):
                    return target
                else:
                    return os.path.join(os.path.dirname(chassis_symlink), target)
            else:
                self.get_logger().warn(f"{chassis_symlink} 不是符号链接")
                return ""
        except Exception as e:
            self.get_logger().warn(f"读取底盘串口失败: {e}")
            return ""

    def _arm_worker_loop(self):
        while self._arm_running:
            cmd_item = self._arm_cmd_queue.get()
            if cmd_item is None:
                break
            func, args, kwargs = cmd_item
            if self.arm_controller is None:
                continue
            try:
                func(*args, **kwargs)
            except Exception as e:
                self.get_logger().warn(f"机械臂指令执行失败: {e}", throttle_duration_sec=2)

    def _async_arm_call(self, func, *args, **kwargs):
        """异步发送机械臂指令；stop 同步发送"""
        if self.arm_controller is None:
            return

        if func == self.arm_controller.stop:
            try:
                while not self._arm_cmd_queue.empty():
                    try:
                        self._arm_cmd_queue.get_nowait()
                    except queue.Empty:
                        break
                func(*args, **kwargs)
                time.sleep(0.005)
                func(*args, **kwargs)
            except Exception:
                pass
            return

        if self._arm_cmd_queue.full():
            try:
                self._arm_cmd_queue.get_nowait()
            except queue.Empty:
                pass
        try:
            self._arm_cmd_queue.put_nowait((func, args, kwargs))
        except queue.Full:
            pass

    def _try_connect_joystick(self):
        if self._joystick_connected:
            return

        device = InputJoystick.get_gamepad(0)
        if device is not None:
            self.joystick.set_device(device)
            self._joystick_connected = True
            self.joystick_thread = threading.Thread(target=self._joystick_read_loop)
            self.joystick_thread.daemon = True
            self.joystick_thread.start()
            self._speed_timer = self.create_timer(0.1, self.send_speed_timer)
            if hasattr(self, '_joystick_check_timer') and self._joystick_check_timer is not None:
                self._joystick_check_timer.cancel()
                self._joystick_check_timer = None
            self.get_logger().info("手柄连接成功")
            self._joystick_warn_logged = False
        else:
            if not self._joystick_warn_logged:
                self._joystick_warn_logged = True

    def _joystick_read_loop(self):
        try:
            while self._joystick_connected:
                self.joystick.monitoring()
                time.sleep(0.01)
        except Exception as e:
            self.get_logger().warn(f"手柄读取异常: {e}")
        finally:
            self._handle_joystick_disconnect()

    def _handle_joystick_disconnect(self):
        self._joystick_connected = False
        self.linear_speed_x = 0.0
        self.linear_speed_y = 0.0
        self.angular_speed = 0.0
        self.publish_speed()
        while not self._arm_cmd_queue.empty():
            try:
                self._arm_cmd_queue.get_nowait()
            except queue.Empty:
                break
        self._async_arm_call(self.arm_controller.stop)
        if self._speed_timer is not None:
            self._speed_timer.cancel()
            self._speed_timer = None
        if not hasattr(self, '_joystick_check_timer') or self._joystick_check_timer is None:
            self._joystick_check_timer = self.create_timer(2.0, self._try_connect_joystick)
        self.get_logger().warn("手柄已断开，将自动重试连接")

    def register_joystick_events(self):
        @self.joystick.register(hotkey=Hotkey.RIGHT_X_AXIS)
        def RIGHT_X_AXIS(self, value: int):
            if value < 128:
                self.linear_speed_y = self.max_linear_speed
            elif value > 128:
                self.linear_speed_y = -self.max_linear_speed
            else:
                self.linear_speed_y = 0.0
            self.publish_speed()

        @self.joystick.register(hotkey=Hotkey.RIGHT_Y_AXIS)
        def RIGHT_Y_AXIS(self, value: int):
            if value < 128:
                self.linear_speed_x = self.max_linear_speed
            elif value > 128:
                self.linear_speed_x = -self.max_linear_speed
            else:
                self.linear_speed_x = 0.0
            self.publish_speed()

        @self.joystick.register(hotkey=Hotkey.L2, value_filter=lambda v: v == 1)
        def L2(self, value: int):
            self.max_linear_speed = max(0.1, self.max_linear_speed - 0.05)
            self.max_angular_speed = max(0.5, self.max_angular_speed - 0.1)
            self.get_logger().info(f"速度减小: 线速度={self.max_linear_speed:.2f}, 角速度={self.max_angular_speed:.2f}")

        @self.joystick.register(hotkey=Hotkey.R2, value_filter=lambda v: v == 1)
        def R2(self, value: int):
            self.max_linear_speed = min(1.6, self.max_linear_speed + 0.05)
            self.max_angular_speed = min(4.0, self.max_angular_speed + 0.1)
            self.get_logger().info(f"速度增大: 线速度={self.max_linear_speed:.2f}, 角速度={self.max_angular_speed:.2f}")

        @self.joystick.register(hotkey=Hotkey.L1)
        def L1(self, value: int):
            self.angular_speed = self.max_angular_speed if value == 1 else 0.0
            self.publish_speed()

        @self.joystick.register(hotkey=Hotkey.R1)
        def R1(self, value: int):
            self.angular_speed = -self.max_angular_speed if value == 1 else 0.0
            self.publish_speed()

        @self.joystick.register(hotkey=Hotkey.STARTUP, value_filter=lambda v: v == 0)
        def STARTUP(self, value: int):
            self.linear_speed_x = 0.0
            self.linear_speed_y = 0.0
            self.angular_speed = 0.0
            self.publish_speed()
            self._async_arm_call(self.arm_controller.go_home)
            self.get_logger().info("机械臂复位")

        @self.joystick.register(hotkey=Hotkey.Y)
        def Y(self, value: int):
            now = self.get_clock().now().nanoseconds / 1e9
            if value == 1:
                if not self._gripper_active:
                    self._gripper_active = True
                    self._async_arm_call(self.arm_controller.open_gripper, 1)
            else:
                if self._gripper_active and (now - self._last_gripper_stop) > 0.02:
                    self._gripper_active = False
                    self._last_gripper_stop = now
                    self._async_arm_call(self.arm_controller.stop)

        @self.joystick.register(hotkey=Hotkey.A)
        def A(self, value: int):
            now = self.get_clock().now().nanoseconds / 1e9
            if value == 1:
                if not self._gripper_active:
                    self._gripper_active = True
                    self._async_arm_call(self.arm_controller.close_gripper, 1)
            else:
                if self._gripper_active and (now - self._last_gripper_stop) > 0.02:
                    self._gripper_active = False
                    self._last_gripper_stop = now
                    self._async_arm_call(self.arm_controller.stop)

        @self.joystick.register(hotkey=Hotkey.X, value_filter=lambda v: v == 0)
        def X(self, _: int):
            try:
                self._async_arm_call(self.arm_controller.open_suction_pump)
            except NotImplementedError as e:
                print(e)

        @self.joystick.register(hotkey=Hotkey.B, value_filter=lambda v: v == 0)
        def B(self, _: int):
            try:
                self._async_arm_call(self.arm_controller.close_suction_pump)
            except NotImplementedError as e:
                print(e)

        @self.joystick.register(hotkey=Hotkey.HORIZONTAL)
        def HORIZONTAL(self, value: int):
            now = self.get_clock().now().nanoseconds / 1e9
            if value == -1:
                if not self._horizontal_active:
                    self._horizontal_active = True
                    self._async_arm_call(self.arm_controller.set_end_rotate, direction=-1, speed=20)
            elif value == 1:
                if not self._horizontal_active:
                    self._horizontal_active = True
                    self._async_arm_call(self.arm_controller.set_end_rotate, direction=1, speed=20)
            else:
                if self._horizontal_active and (now - self._last_horizontal_stop) > 0.02:
                    self._horizontal_active = False
                    self._last_horizontal_stop = now
                    self._async_arm_call(self.arm_controller.stop)

        @self.joystick.register(hotkey=Hotkey.VERTICAL)
        def VERTICAL(self, value: int):
            now = self.get_clock().now().nanoseconds / 1e9
            if value == -1:
                if not self._vertical_active:
                    self._vertical_active = True
                    self._async_arm_call(self.arm_controller.coordinate, axis=3, direction=1)
            elif value == 1:
                if not self._vertical_active:
                    self._vertical_active = True
                    self._async_arm_call(self.arm_controller.coordinate, axis=3, direction=0)
            else:
                if self._vertical_active and (now - self._last_vertical_stop) > 0.02:
                    self._vertical_active = False
                    self._last_vertical_stop = now
                    self._async_arm_call(self.arm_controller.stop)

        @self.joystick.register(hotkey=Hotkey.LEFT_X_AXIS)
        def LEFT_X_AXIS(self, value: int):
            now = self.get_clock().now().nanoseconds / 1e9
            offset = value - 128

            if abs(offset) < DEADZONE:
                if self._left_x_active != 0 and (now - self._last_left_x_stop) > 0.02:
                    self._left_x_active = 0
                    self._last_left_x_stop = now
                    self._async_arm_call(self.arm_controller.stop)
                return

            if offset > 0 and self._left_x_active != 1:
                self._left_x_active = 1
                self._async_arm_call(self.arm_controller.coordinate, axis=2, direction=0)
            elif offset < 0 and self._left_x_active != -1:
                self._left_x_active = -1
                self._async_arm_call(self.arm_controller.coordinate, axis=2, direction=1)

        @self.joystick.register(hotkey=Hotkey.LEFT_Y_AXIS)
        def LEFT_Y_AXIS(self, value: int):
            now = self.get_clock().now().nanoseconds / 1e9
            offset = value - 128

            if abs(offset) < DEADZONE:
                if self._left_y_active != 0 and (now - self._last_left_y_stop) > 0.02:
                    self._left_y_active = 0
                    self._last_left_y_stop = now
                    self._async_arm_call(self.arm_controller.stop)
                return

            if offset < 0 and self._left_y_active != 1:
                self._left_y_active = 1
                self._async_arm_call(self.arm_controller.coordinate, axis=1, direction=1)
            elif offset > 0 and self._left_y_active != -1:
                self._left_y_active = -1
                self._async_arm_call(self.arm_controller.coordinate, axis=1, direction=0)

    def publish_speed(self):
        twist_stamp = TwistStamped()
        twist_stamp.header.stamp = self.get_clock().now().to_msg()
        twist_stamp.header.frame_id = "base_link"
        twist_stamp.twist.linear.x = self.linear_speed_x
        twist_stamp.twist.linear.y = self.linear_speed_y
        twist_stamp.twist.angular.z = self.angular_speed
        self.cmd_vel_publisher.publish(twist_stamp)

    def send_speed_timer(self):
        self.publish_speed()


def main(args=None):
    rclpy.init(args=args)
    node = JoystickController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._arm_running = False
        node._joystick_connected = False
        if node._speed_timer is not None:
            node._speed_timer.cancel()
            node._speed_timer = None
        if hasattr(node, '_joystick_check_timer') and node._joystick_check_timer is not None:
            node._joystick_check_timer.cancel()
            node._joystick_check_timer = None
        try:
            node._arm_cmd_queue.put_nowait(None)
        except queue.Full:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()