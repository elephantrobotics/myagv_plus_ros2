#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from myagv_plus_msgs.srv import QueryDevice

SET_PUMP_STATE = 0x43  # 吸泵开/关
SET_PUMP_IO    = 0x44  # 吸泵 IO 直接控制


class PumpClient(Node):
    def __init__(self):
        super().__init__('logistics_lite_pump_client')
        self.cli_query = self.create_client(QueryDevice, 'query_device')
        while not self.cli_query.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /query_device service...')

    def _set(self, cmd_id, payload):
        # 发一条写命令,收到合法返回返回 1,失败返回 0
        req = QueryDevice.Request()
        req.cmd_id = cmd_id
        req.payload = bytes((list(payload) + [0] * 8)[:8])
        future = self.cli_query.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        res = future.result()
        if res is None or not res.success:
            self.get_logger().error(f"cmd 0x{cmd_id:02X} call failed")
            return 0
        data = list(res.data)
        if len(data) < 6 or data[3] != cmd_id:
            self.get_logger().error(f"cmd 0x{cmd_id:02X} bad response: {data}")
            return 0
        return 1

    def set_pump_state(self, state):
        # 吸泵开关,1 开 0 关
        # pump.set_pump_state(1)
        return self._set(SET_PUMP_STATE, [state])

    def set_pump_io(self, pin, state):
        # 直接控制吸泵 IO,pin 取 2 或 5;state 0 低电平 / 1 高电平
        # 最新吸泵 5 号脚低电平工作、高电平关闭,破真空时手动发时序用
        # pump.set_pump_io(5, 0)
        return self._set(SET_PUMP_IO, [pin, state])
