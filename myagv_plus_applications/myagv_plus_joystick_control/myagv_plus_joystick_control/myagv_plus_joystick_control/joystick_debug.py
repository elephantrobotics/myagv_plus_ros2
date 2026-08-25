#!/usr/bin/env python3
"""
手柄按键调试脚本
显示所有手柄事件，帮助诊断按键映射问题
"""
from inputs import devices, get_gamepad

def main():
    print("正在查找手柄...")
    gamepads = devices.gamepads
    
    if not gamepads:
        print("未检测到手柄！")
        return
    
    print(f"检测到 {len(gamepads)} 个手柄:")
    for i, gamepad in enumerate(gamepads):
        print(f"  {i}: {gamepad.name}")
    
    # 使用第一个手柄
    gamepad = gamepads[0]
    print(f"\n使用手柄: {gamepad.name}")
    print("按下任意按键查看事件...")
    print("按 Ctrl+C 退出")
    
    try:
        while True:
            events = gamepad.read()
            for event in events:
                print(f"事件类型: {event.ev_type}, 代码: {event.code}, 值: {event.state}")
    except KeyboardInterrupt:
        print("\n退出调试")

if __name__ == "__main__":
    main()