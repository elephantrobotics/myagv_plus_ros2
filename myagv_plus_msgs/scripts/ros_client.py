#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from myagv_plus_msgs.srv import *

GET_MODIFY_VERSION    = 0x01  # 读次固件版本号
GET_SYSTEM_VERSION    = 0x02  # 读主固件版本号
GET_ROBOT_STATUS      = 0x05  # 读整机状态(电池/陀螺仪/电量)
MOTOR_POWER_ON        = 0x10  # 电机上电/断电
IS_MOTOR_POWERED      = 0x12  # 读电机供电状态
SET_AUTO_REPORT       = 0x23  # 开/关自动上发
GET_AUTO_REPORT       = 0x24  # 读自动上发开关
SET_LED_COLOR         = 0x34  # 设灯带颜色(DIY)
SET_LED_MODE          = 0x3A  # 设灯带模式(电量/DIY)
SET_OUT_IO            = 0x40  # 设输出引脚电平
GET_IN_IO             = 0x41  # 读输入引脚电平
SET_FAN_STATE         = 0x42  # 风扇开/关
SET_PUMP_STATE        = 0x43  # 吸泵开/关
SET_PUMP_IO           = 0x44  # 吸泵 IO 直接控制


class AGVIOClient(Node):
    def __init__(self):
        super().__init__('ros_client')
        self.cli_query = self.create_client(QueryDevice, 'query_device')
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)
        self._wait_for_services()

    def _wait_for_services(self):
        while not self.cli_query.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /query_device service...')

    def _call_service(self, client, request):
        future = client.call_async(request)
        self._executor.spin_until_future_complete(future, timeout_sec=5.0)
        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error(f'Service call failed: {future.exception()}')
            return None

    def _send(self, cmd_id: int, payload=None):
        # 发一条命令,返回里取出 8 字节数据区(去掉帧头/长度/指令/CRC),失败返回 None
        # 返回帧: [0xFE, 0xFE, 长度, 指令, 数据x8, CRC高, CRC低]
        req = QueryDevice.Request()
        req.cmd_id = cmd_id
        req.payload = bytes(self._pad8(payload or []))

        res = self._call_service(self.cli_query, req)
        if res is None or not res.success:
            self.get_logger().error(f"cmd 0x{cmd_id:02X} call failed")
            return None

        data = list(res.data)
        if len(data) < 6:
            self.get_logger().error("Invalid response frame length")
            return None
        if data[3] != cmd_id:
            self.get_logger().error(f"Command mismatch: expected 0x{cmd_id:02X}, got 0x{data[3]:02X}")
            return None

        return data[4:-2]

    def _pad8(self, vals):
        # 数据区固定 8 字节,不足补 0,超出截断
        p = list(vals)[:8]
        return p + [0] * (8 - len(p))

    def _set(self, cmd_id: int, payload=None) -> int:
        # 写类命令,成功(收到合法返回)返回 1,失败返回 0
        return 1 if self._send(cmd_id, payload) is not None else 0

    # --- System & version ---

    def get_system_version(self):
        # 读取主固件版本号,原始值除以 10
        # agv.get_system_version()  ->  1.5
        data = self._send(GET_SYSTEM_VERSION)
        return data[0] / 10.0 if data else None

    def get_modify_version(self):
        # 读取次固件版本号,同样除以 10,主要用来区分测试迭代
        # agv.get_modify_version()  ->  0.2
        data = self._send(GET_MODIFY_VERSION)
        return data[0] / 10.0 if data else None

    def get_robot_status(self):
        # 读硬件异常状态,返回 [电池, 陀螺仪, 电量]
        # 电池/陀螺仪: 0 正常 / 1 异常;电量: 0 正常 / 1 警告(≤19.6V) / 2 低压(<19V)
        # agv.get_robot_status()  ->  [0, 0, 0]
        data = self._send(GET_ROBOT_STATUS)
        if not data or len(data) < 3:
            return None
        return [data[0], data[1], data[2]]

    # --- Motor power ---

    def power_on(self) -> int:
        # 开启机器人(电机供电/继电器通电),对应 0x10 state=1
        # agv.power_on()  ->  1
        return self._set(MOTOR_POWER_ON, [1])

    def power_off(self) -> int:
        # 关闭机器人(断开电机供电/继电器),对应 0x10 state=0
        # agv.power_off()  ->  1
        return self._set(MOTOR_POWER_ON, [0])

    def is_power_on(self):
        # 查机器人电源是否开启,1=开机 0=关机
        # agv.is_power_on()  ->  1
        data = self._send(IS_MOTOR_POWERED)
        return data[0] if data else None

    # --- Auto report ---

    def set_auto_report_state(self, state: int) -> int:
        # 开关自动上发,1 开 0 关
        # 开了之后底板每 50ms 主动上报电池/陀螺仪,走 /imu、/voltage 等话题
        # agv.set_auto_report_state(1)
        return self._set(SET_AUTO_REPORT, [state])

    def get_auto_report_state(self):
        # 查自动上发当前是否开启,1=开 0=关
        # agv.get_auto_report_state()  ->  1
        data = self._send(GET_AUTO_REPORT)
        return data[0] if data else None

    # --- LED ---

    def set_led_color(self, brightness: int, color: tuple) -> int:
        # 设置灯带颜色,要先 set_led_mode(1) 切到 DIY 模式才生效
        # brightness 0-255,color 是 (r, g, b),各 0-255
        # agv.set_led_color(100, (255, 0, 0))  红色,亮度100
        r, g, b = color
        payload = [brightness, r, g, b]
        return self._set(SET_LED_COLOR, payload)

    def set_led_mode(self, mode: int) -> int:
        # 切灯带模式,0 跟随电量显示(默认)/ 1 DIY 自定义颜色
        # agv.set_led_mode(1)
        return self._set(SET_LED_MODE, [mode])

    # --- Digital IO & fan ---

    def set_pin_output(self, pin: int, state: int) -> int:
        # 设置输出引脚电平,state 1 高 0 低
        # pin 1-6 对应 base 板丝印 24/22/23/27/18/17,0 表示全部
        # agv.set_pin_output(1, 1)
        return self._set(SET_OUT_IO, [pin, state])

    def get_pin_input(self, pin: int):
        # 读输入引脚电平,pin 1-6 对应 base 板丝印 7/11/8/9/25/10
        # 返回该引脚电平 0/1;不存在的引脚返回 -1
        # pin=0 时一次返回 1-6 号引脚状态的列表
        # agv.get_pin_input(1)  ->  0
        data = self._send(GET_IN_IO, [pin])
        if not data or len(data) < 2:
            return -1
        if pin == 0:
            return list(data[1:7])
        if data[0] != pin:  # 固件不认识的引脚:byte1 回 1
            return -1
        return data[1]

    def set_fan_state(self, state: int) -> int:
        # 风扇开关,1 开 0 关,默认开
        # agv.set_fan_state(1)
        return self._set(SET_FAN_STATE, [state])

    def set_pump_state(self, state: int) -> int:
        # 吸泵开关,1 开 0 关
        # agv.set_pump_state(1)
        return self._set(SET_PUMP_STATE, [state])

    def set_pump_io(self, pin: int, state: int) -> int:
        # 直接控制吸泵 IO,pin 取 2 或 5(对应吸泵2、5)
        # state 0 低电平 / 1 高电平;最新吸泵 5 号脚低电平工作、高电平关闭
        # agv.set_pump_io(5, 0)
        return self._set(SET_PUMP_IO, [pin, state])


def main(args=None):
    rclpy.init(args=args)
    client = AGVIOClient()

    print("0x01 get_modify_version:", client.get_modify_version())      # [读] 无参,返回次版本号
    print("0x02 get_system_version:", client.get_system_version())      # [读] 无参,返回主版本号
    print("0x05 get_robot_status:", client.get_robot_status())          # [读] 无参,返回 [电池, 陀螺仪, 电量]
    # print("0x10 power_on:", client.power_on())                          # [写] 开机(电机供电)
    # print("0x10 power_off:", client.power_off())                        # [写] 关机(断电)
    # print("0x12 is_power_on:", client.is_power_on())                    # [读] 无参,返回 1=开机/0=关机
    # print("0x23 set_auto_report:", client.set_auto_report_state(1))     # [写] state: 1=开 0=关
    # print("0x24 get_auto_report:", client.get_auto_report_state())      # [读] 无参,返回 1=开/0=关
    # print("0x34 set_led_color:", client.set_led_color(100, (255, 0, 0)))  # [写] brightness 0-255, color=(R,G,B)各0-255;红(255,0,0)/绿(0,255,0)/蓝(0,0,255)/白(255,255,255);先切 DIY
    # print("0x3A set_led_mode:", client.set_led_mode(1))                 # [写] mode: 0=电量显示 1=DIY自定义
    # print("0x40 set_pin_output:", client.set_pin_output(1, 1))          # [写] pin 1-6→丝印24/22/23/27/18/17, state: 1=高 0=低
    # print("0x41 get_pin_input:", client.get_pin_input(0))               # [读] pin 1-6→丝印7/11/8/9/25/10(单个引脚)
    # print("0x41 get_pin_input:", client.get_pin_input(0))               # [读] pin=0 读全部1-6,返回列表(规范外扩展)
    # print("0x42 set_fan_state:", client.set_fan_state(1))               # [写] state: 1=开 0=关
    # print("0x43 set_pump_state:", client.set_pump_state(1))             # [写] state: 1=开 0=关
    # print("0x44 set_pump_io:", client.set_pump_io(5, 0))                # [写] pin 2/5, state: 0=低电平 1=高电平(最新吸泵5脚低电平工作)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
