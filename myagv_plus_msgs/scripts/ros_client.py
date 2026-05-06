#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from myagv_plus_msgs.srv import *

GET_MODIFY_VERSION = 0x01
GET_SYSTEM_VERSION = 0x02
GET_ROBOT_STATUS = 0x05
SET_AUTO_REPORT   = 0x23
GET_AUTO_REPORT   = 0x24

class AGVIOClient(Node):
    def __init__(self):
        super().__init__('ros_client')

        # Create service client
        self.cli_led_output = self.create_client(SetLedColor, 'set_led_color')
        self.cli_led_mode = self.create_client(SetLedMode, 'set_led_mode')
        self.cli_query = self.create_client(QueryDevice, 'query_device')

        # Wait until all services are available
        self._wait_for_services()

    def _wait_for_services(self):
        """Wait for all services to become available."""
        clients = [
            self.cli_led_output,
            self.cli_led_mode,
            self.cli_query
        ]

        for cli in clients:
            while not cli.wait_for_service(timeout_sec=1.0):
                pass

    def _call_service(self, client, request):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error(f'Service call failed: {future.exception()}')
            return None

    def _query(self, cmd_id: int):
        req = QueryDevice.Request()
        req.cmd_id = cmd_id

        res = self._call_service(self.cli_query, req)

        if res is None or not res.success:
            self.get_logger().error(f"Query cmd {cmd_id} failed")
            return None

        data = list(res.data)

        if len(data) < 6:
            self.get_logger().error("Invalid response length")
            return None

        if data[3] != cmd_id:
            self.get_logger().error(f"CMD mismatch: expect {cmd_id}, got {data[3]}")
            return None

        return data[4:-2]  # payload

    def _query_and_parse(self, cmd_id: int, parser):
        payload = self._query(cmd_id)
        if payload is None:
            return None

        try:
            return parser(payload)
        except Exception as e:
            self.get_logger().error(f"Parse failed (cmd={cmd_id}): {e}")
            return None

    # ------------------------------
    # Set LED color
    # ------------------------------
    def set_led_color(self,
                      position: int,
                      brightness: int,
                      r: int,
                      g: int,
                      b: int) -> bool:
        """Set LED RGB color and brightness."""
        req = SetLedColor.Request()
        req.position = position
        req.brightness = brightness
        req.r = r
        req.g = g
        req.b = b

        res = self._call_service(self.cli_led_output, req)
        return res.success if res else False

    # ------------------------------
    # Set LED mode
    # ------------------------------
    def set_led_mode(self, mode: bool) -> bool:
        """Set LED mode (True/False)."""
        req = SetLedMode.Request()
        req.mode = mode

        res = self._call_service(self.cli_led_mode, req)
        return res.success if res else False

    def get_modify_version(self):
        return self._query_and_parse(
            GET_MODIFY_VERSION,
            lambda p: bytes(p).decode('ascii').strip('\x00')
        )

    def get_system_version(self):
        return self._query_and_parse(
            GET_SYSTEM_VERSION,
            lambda p: bytes(p).decode('ascii').strip('\x00')
        )

    def get_robot_status(self):
        def parser(p):
            if len(p) < 6:
                return None
            return {
                "battery_status": p[0],
                "imu_status": p[1],
                "battery_level": p[2],
                "charging": p[3],
                "voltage": p[4] / 10.0,
                "backup_voltage": p[5] / 10.0
            }

        return self._query_and_parse(GET_ROBOT_STATUS, parser)

    def get_auto_report_status(self):
        return self._query_and_parse(
            GET_AUTO_REPORT,
            lambda p: bool(p[0]) if len(p) > 0 else None
        )

def main(args=None):
    rclpy.init(args=args)
    client = AGVIOClient()

    ################
    # Example usage:
    ################

    client.set_led_mode(True)
    client.set_led_color(0, 100, 255, 0, 0)
    client.set_led_color(1, 100, 0, 255, 0)

    print("Modify Version:", client.get_modify_version())
    print("System Version:", client.get_system_version())
    print("Robot Status:", client.get_robot_status())
    print("Auto Report:", client.get_auto_report_status())

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()