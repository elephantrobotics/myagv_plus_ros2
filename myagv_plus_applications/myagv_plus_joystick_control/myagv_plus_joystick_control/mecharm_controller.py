#!/usr/bin/env python
# -*- coding: UTF-8 -*-
from pymycobot import MechArm270
from ros_client import AGVIOClient  # 新增导入


class BaseControllerApi:
    def open_suction_pump(self):
        raise NotImplementedError("Not implemented")

    def close_suction_pump(self):
        raise NotImplementedError("Not implemented")

    def set_end_rotate(self, direction: int, speed: float):
        raise NotImplementedError("Not implemented")

    def open_gripper(self, speed: int = 1):
        raise NotImplementedError("Not implemented")

    def close_gripper(self, speed: int = 1):
        raise NotImplementedError("Not implemented")

    def coordinate(self, axis: int, direction: int):
        raise NotImplementedError("Not implemented")

    def go_home(self):
        raise NotImplementedError("Not implemented")

    def stop(self):
        raise NotImplementedError("Not implemented")

    def set_fresh_mode(self, mode: int):
        raise NotImplementedError("Not implemented")


class MechArm270Controller(BaseControllerApi):
    def __init__(self, port: str, agv_io_client=None, debug: bool = False):
        """
        :param port: 机械臂串口
        :param agv_io_client: AGVIOClient实例，用于IO控制（吸泵等）
        :param debug: 调试模式
        """
        self._cobot = MechArm270(
            port,
            baudrate=115200,
            debug=debug,
            thread_lock=False
        )
        self.set_fresh_mode(1)
        
        # 保存AGV IO客户端
        self._agv_io = agv_io_client
        
        # 修复pymycobot读取问题
        self._cobot._read = lambda *args, **kwargs: b''
        if hasattr(self._cobot, '_lock'):
            self._cobot._lock = None

    def open_suction_pump(self):
        """开启吸泵 - 使用AGV IO"""
        if self._agv_io is None:
            print(" # (Warning) AGVIOClient not available, cannot control pump")
            return
            
        try:
            self._agv_io.set_pump_state(1)
        except Exception as e:
            print(f" # (Error) MechArm270 open suction pump: {e}")

    def close_suction_pump(self):
        """关闭吸泵 - 使用AGV IO"""
        if self._agv_io is None:
            print(" # (Warning) AGVIOClient not available, cannot control pump")
            return
            
        try:
            self._agv_io.set_pump_state(0)
        except Exception as e:
            print(f" # (Error) MechArm270 close suction pump: {e}")

    def set_end_rotate(self, direction: int, speed: float):
        jog_dir = 1 if direction > 0 else 0
        self._cobot.jog_angle(6, jog_dir, int(speed), _async=True)

    def open_gripper(self, speed: int = 1):
        self._cobot.set_gripper_value(100, speed, 1)

    def close_gripper(self, speed: int = 1):
        self._cobot.set_gripper_value(0, speed, 1)

    def coordinate(self, axis: int, direction: int):
        increment = 1 if direction == 1 else 0
        self._cobot.jog_coord(axis, increment, 10, _async=True)

    def go_home(self):
        self._cobot.send_angles([0, 0, 0, 0, 90, 0], 10, _async=True)

    def stop(self):
        self._cobot.stop()

    def set_fresh_mode(self, mode: int):
        self._cobot.set_fresh_mode(mode)


class UndefinedController(BaseControllerApi):
    def open_suction_pump(self):
        pass

    def close_suction_pump(self):
        pass

    def set_end_rotate(self, direction: int, speed: float):
        pass

    def open_gripper(self, speed: int = 1):
        pass

    def close_gripper(self, speed: int = 1):
        pass

    def coordinate(self, axis: int, direction: int):
        pass

    def go_home(self):
        pass

    def stop(self):
        pass
        
    def set_fresh_mode(self, mode: int):  
        pass