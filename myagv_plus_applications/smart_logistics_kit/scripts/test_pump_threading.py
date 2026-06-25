#!/usr/bin/env python3
# 吸泵并发自旋回归测试
#
# 验证在 main.py 的线程模型下,底部协议吸泵控制(AGVIOClient)能稳定工作:
#   - 主线程持续 spin 任务节点
#   - worker 线程构造 AGVIOClient 并反复开关吸泵
# 这复刻了物流主流程中 get_arm() 与吸泵调用所处的并发上下文。
#
# 用法: python3 scripts/test_pump_threading.py
# 前置: esp32 节点已运行并提供 /query_device 服务;已配置 PYTHONPATH 使 ros_client 可导入

import threading
import time

import rclpy
from rclpy.node import Node

CYCLES = 5
HOLD_SECONDS = 1.0


def pump_worker():
    from ros_client import AGVIOClient
    pump = AGVIOClient()  # worker 线程里构造,阻塞等 query_device,同 main 的 get_arm
    ok = 0
    total = CYCLES * 2
    try:
        for i in range(CYCLES):
            r_on = pump.set_pump_state(1)
            print(f'[worker] cycle {i} pump ON  -> {r_on}')
            ok += 1 if r_on == 1 else 0
            time.sleep(HOLD_SECONDS)
            r_off = pump.set_pump_state(0)
            print(f'[worker] cycle {i} pump OFF -> {r_off}')
            ok += 1 if r_off == 1 else 0
            time.sleep(HOLD_SECONDS)
    finally:
        if ok == total:
            print(f'[worker] PASS: {ok}/{total} pump calls succeeded under main-like threading.')
        else:
            print(f'[worker] FAIL: only {ok}/{total} pump calls succeeded.')
        pump.destroy_node()
        rclpy.shutdown()


def main():
    rclpy.init()
    mission_like = Node('mission_like')  # 模拟主线程持续 spin 的任务节点
    threading.Thread(target=pump_worker, daemon=True).start()
    try:
        rclpy.spin(mission_like)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f'[main] spin ended: {error}')
    mission_like.destroy_node()


if __name__ == '__main__':
    main()
