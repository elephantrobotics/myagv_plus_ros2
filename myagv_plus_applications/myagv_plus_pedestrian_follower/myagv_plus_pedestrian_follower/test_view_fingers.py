# !/usr/bin/env python
# -*- coding: UTF-8 -*-
"""
需要安装 cvzone和mediapipe库，注意这两个库会更新numpy的版本
"""
import threading
import time
from pymycobot import *
import cv2
from cvzone.HandTrackingModule import HandDetector

cap = cv2.VideoCapture(3)


# 创建手部检测器对象
detector = HandDetector(
    detectionCon=0.8,  # 检测置信度阈值（0~1，越高越严格）
    maxHands=2  # 最多检测的手部数量
)

number = 0

while True:
    success, img = cap.read()  # 读取摄像头画面
    if not success:
        break

    # 检测手部
    hands, img = detector.findHands(img)  # hands 包含检测结果

    if hands:
        # 遍历每只检测到的手
        for hand in hands:
            # 获取手部信息
            handType = hand["type"]  # 手型（"Left" 或 "Right"）
            lmList = hand["lmList"]  # 21个关键点的 (x, y) 坐标列表
            bbox = hand["bbox"]  # 边界框 (x, y, w, h)
            center = hand["center"]  # 手掌中心点坐标
            fingers = detector.fingersUp(hand)  # 手指状态（[0,1,0,1,1] 表示食指、无名指、小指竖起）
            print(fingers)

            if fingers == [0, 1, 1, 0, 0]:
                number += 1
                if number == 110:
                    print("开始拍照咯！")
                    for i in range(1, 6):
                        print("拍照倒计时：", 6 - i)
                        time.sleep(1)
                    print("拍照成功！")
                    number = 0

    # 显示画面
    cv2.imshow("Hand Tracking", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):  # 按 Q 退出
        break

cap.release()
cv2.destroyAllWindows()
