#!/usr/bin/env python3
"""High-level ROS client for the myAGV Plus ESP32 serial protocol.

The C++ driver exposes one generic QueryDevice service:

    cmd_id + payload -> raw serial response frame

This client converts convenient Python methods into that service format and
validates the frame returned from the ESP32.
"""

from __future__ import annotations

from typing import Callable, Iterable, Optional
import enum
import rclpy
from rclpy.node import Node

from myagv_plus_msgs.srv import QueryDevice

class ProtocolCode(enum.Enum):
    GET_MODIFY_VERSION = 0x01
    GET_SYSTEM_VERSION = 0x02
    GET_ROBOT_STATUS = 0x05
    MOTOR_POWER_ON = 0x10
    IS_MOTOR_POWERED = 0x12
    SET_AUTO_REPORT = 0x23
    GET_AUTO_REPORT = 0x24
    SET_LED_COLOR = 0x34
    SET_LED_MODE = 0x3A
    SET_OUT_IO = 0x40
    GET_IN_IO = 0x41
    SET_FAN_STATUS = 0x42

    FRAME_HEADER = [0xFE, 0xFE, 0x0B]
    RESPONSE_PAYLOAD_SIZE = 8
    RESPONSE_FRAME_SIZE = len(FRAME_HEADER) + 1 + RESPONSE_PAYLOAD_SIZE + 2

    def equal(self, other):
        if isinstance(other, ProtocolCode):
            return self.value == other.value
        else:
            return self.value == other

class AGVClient(Node):
    def __init__(self, service_name: str = "query_device"):
        super().__init__("ros_client")
        self.cli_query = self.create_client(QueryDevice, service_name)

        self.get_logger().info(f"Waiting for service '{service_name}'...")
        while rclpy.ok() and not self.cli_query.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Service '{service_name}' not available, waiting...")

    def _call_service(self, client, request, timeout_sec: float = 5.0):
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        if not future.done():
            self.get_logger().error("Service call timed out")
            return None

        if future.result() is not None:
            return future.result()

        self.get_logger().error(f"Service call failed: {future.exception()}")
        return None

    @staticmethod
    def _format_hex(data: Iterable[int]) -> str:
        return " ".join(f"{value & 0xFF:02X}" for value in data)

    def query_device(
        self,
        cmd_id: int,
        payload: Optional[Iterable[int]] = None,
        timeout_sec: float = 5.0,
        validate_crc: bool = True,
    ) -> Optional[list[int]]:
        """Call QueryDevice and return the raw response frame."""
        cmd_id = self._byte(cmd_id, "cmd_id")
        payload_bytes = [self._byte(value, "payload byte") for value in (payload or [])]

        if len(payload_bytes) > RESPONSE_PAYLOAD_SIZE:
            raise ValueError(f"payload must contain at most {RESPONSE_PAYLOAD_SIZE} bytes")

        req = QueryDevice.Request()
        req.cmd_id = cmd_id
        req.payload = payload_bytes

        res = self._call_service(self.cli_query, req, timeout_sec=timeout_sec)
        if res is None or not res.success:
            self.get_logger().error(f"QueryDevice failed for cmd 0x{cmd_id:02X}")
            return None

        frame = list(res.data)
        if validate_crc and not self._validate_frame(cmd_id, frame):
            return None

        return frame

    def _payload(self, cmd_id: int, payload: Optional[Iterable[int]] = None) -> Optional[list[int]]:
        frame = self.query_device(cmd_id, payload)
        if frame is None:
            return None
        return frame[4:-2]

    def _query_and_parse(
        self,
        cmd_id: int,
        parser: Callable[[list[int]], object],
        payload: Optional[Iterable[int]] = None,
    ):
        response_payload = self._payload(cmd_id, payload)
        if response_payload is None:
            return None

        try:
            return parser(response_payload)
        except (IndexError, ValueError, TypeError) as exc:
            self.get_logger().error(f"Parse failed for cmd 0x{cmd_id:02X}: {exc}")
            return None

    def _ack(self, cmd_id: int, payload: Optional[Iterable[int]] = None) -> bool:
        response_payload = self._payload(cmd_id, payload)
        if response_payload is None:
            return False

        success = response_payload[0] == 0x01
        if not success:
            self.get_logger().error(
                f"Command 0x{cmd_id:02X} failed with status 0x{response_payload[0]:02X}"
            )
        return success

    def _merge(self, cmd_id: int, *payload: int, parser=None):
        response_payload = self._payload(cmd_id, payload)
        if response_payload is None:
            return None

        if parser is None:
            return response_payload

        return response_payload

    @classmethod
    def _parsing_data(cls, genre, reply_data):
        if not reply_data:
            return None

        if ProtocolCode.GET_SYSTEM_VERSION.equal(genre):
            return reply_data[0] / 10

        if ProtocolCode.GET_MOTOR_TEMPERATURE.equal(genre):
            return list(data / 10 for data in reply_data)

        if ProtocolCode.GET_ROBOT_STATUS.equal(genre):
            pass

        if ProtocolCode.GET_MOTOR_ENABLE_STATUS.equal(genre):
            return list(reply_data[:4])

        if ProtocolCode.GET_INPUT_IO.equal(genre):
            if reply_data[0] == 255:
                return -1
            return reply_data[1]

        return reply_data[0]

    # API methods for specific commands
    def get_modify_version(self) -> Optional[int]:
        return self._query_and_parse(GET_MODIFY_VERSION, lambda payload: payload[0])

    def get_system_version(self) -> Optional[int]:
        return self._query_and_parse(GET_SYSTEM_VERSION, lambda payload: payload[0])

    def get_robot_status(self) -> Optional[dict[str, object]]:
        def parser(payload: list[int]) -> dict[str, object]:
            return {
                "battery_status": payload[0],
                "imu_status": payload[1],
                "battery_level": payload[2],
                "charging": bool(payload[3]),
                "voltage": payload[4] / 10.0,
                "backup_voltage": payload[5] / 10.0,
            }

        return self._query_and_parse(GET_ROBOT_STATUS, parser)

    def motor_power_on(self, enable: bool = True) -> bool:
        return self._ack(MOTOR_POWER_ON, [0x01 if enable else 0x00])

    def is_motor_powered(self) -> Optional[bool]:
        return self._query_and_parse(IS_MOTOR_POWERED, lambda payload: bool(payload[0]))

    def set_auto_report(self, enable: bool) -> bool:
        return self._ack(SET_AUTO_REPORT, [0x01 if enable else 0x00])

    def get_auto_report_status(self) -> Optional[bool]:
        return self._query_and_parse(GET_AUTO_REPORT, lambda payload: bool(payload[0]))

    def set_led_color(self, position: int, brightness: int, color: tuple) -> int:
        payload = [
            self._byte(position, "position"),
            self._byte(brightness, "brightness"),
            self._byte(color[0], "color R"),
            self._byte(color[1], "color G"),
            self._byte(color[2], "color B"),
        ]
        return self._merge(SET_LED_COLOR, payload)

    def set_led_mode(self, mode: int) -> int:
        return self._merge(SET_LED_MODE, [mode])

    def set_pin_output(self, pin: int, state: int) -> bool:
        payload = [
            self._byte(pin, "pin"),
            self._byte(state, "state"),
        ]
        return self._merge(SET_OUT_IO, payload)

    def get_pin_input(self, pin: int) -> int:
        return self._query_and_parse(GET_IN_IO, lambda payload: payload[0], [self._byte(pin, "pin")])

    def set_fan_status(self, state: int) -> int:
        return self._merge(SET_FAN_STATUS, [state])


def main(args=None):
    rclpy.init(args=args)
    client = AGVClient()

    try:
        print("system version:", client.get_system_version())
        print("modify version:", client.get_modify_version())
        print("Robot Status:", client.get_robot_status())
        client.get_auto_report_status()
        client.set_led_color(0, 255, 255, 0, 0)
        client.set_led_mode(True)
        client.set_fan_status(0)
    finally:
        client.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()