#!/usr/bin/env python3

import sys
import time

import cv2

from smart_logistics_kit_lite.QRCodeScanner import (
    QRCodeScanner, VISUAL_LOW_EDGE_MAX, VISUAL_LOW_TVEC_Z_MIN)


PICK_WATCH = [94.13, 20.2, -21.88, 0.96, 90.79, 4.57]
PICK_WATCH = [94.13, 30.2, -25.88, 0.96, 85.79, 4.57]
ARM_PORT = '/dev/ttyACM0'
ARM_BAUDRATE = 115200
ARM_SPEED = 30
ARM_SETTLE = 2.5
QR_CAMERA = '/dev/video1'
PRINT_INTERVAL = 0.5
WINDOW = 'QR Live View'


def move_to_pick_watch():
    from smart_logistics_kit_lite.arm_controller import MechArm270Control
    arm = MechArm270Control(
        port=ARM_PORT,
        baudrate=ARM_BAUDRATE,
        qr_camera=QR_CAMERA,
        qr_show_window=False)
    print(f'移动到 PICK_WATCH = {PICK_WATCH}')
    arm.mc.send_angles([float(a) for a in PICK_WATCH], ARM_SPEED)
    time.sleep(ARM_SETTLE)
    try:
        actual = arm.mc.get_angles()
    except Exception:
        actual = None
    if isinstance(actual, (list, tuple)) and len(actual) >= 6:
        print('实际到位角度 = [%s]' % ', '.join('%.2f' % float(a) for a in actual[:6]))
    return arm


def put(frame, text, row, color=(0, 255, 0)):
    cv2.putText(frame, text, (10, 24 + row * 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)


def main():
    arm = None
    if '--no-arm' not in sys.argv:
        try:
            arm = move_to_pick_watch()
        except Exception as error:
            print(f'机械臂初始化失败({error})，转为纯相机模式')

    scanner = QRCodeScanner(video_device=QR_CAMERA, show_window=False)
    print(f'阈值: VISUAL_LOW_EDGE_MAX={VISUAL_LOW_EDGE_MAX} '
          f'VISUAL_LOW_TVEC_Z_MIN={VISUAL_LOW_TVEC_Z_MIN}')
    print('窗口按 q / ESC 退出，终端 Ctrl-C 也可以\n')

    hit = 0
    total = 0
    last_print = 0.0

    try:
        while True:
            ok, frame = scanner.cap.read()
            if not ok:
                print('读帧失败')
                break

            scanner.clear_visual_height()
            result = scanner.scan_qrcode_from_camera(frame)
            view = result[0] if result else frame

            edge = scanner.last_visual_edge_median
            tvec_z = scanner.last_visual_tvec_z
            found = edge is not None and tvec_z is not None

            total += 1
            if found:
                hit += 1
            rate = 100.0 * hit / total

            if found:
                line = 'edge_med=%.1f tvec_z=%.4f level=%s pick_z=%s' % (
                    edge, tvec_z, scanner.last_visual_level, scanner.last_pick_height)
            else:
                line = 'no measurement (need a full 4-point QR)'

            put(view, 'DETECT: %s' % ('YES' if found else 'NO'), 0,
                (0, 255, 0) if found else (0, 0, 255))
            put(view, line, 1, (0, 255, 0) if found else (0, 0, 255))
            put(view, 'hit=%d/%d (%.0f%%)' % (hit, total, rate), 2, (255, 255, 0))
            put(view, 'thr: edge<=%.2f and tvec_z>=%.4f -> low(50)'
                % (VISUAL_LOW_EDGE_MAX, VISUAL_LOW_TVEC_Z_MIN), 3, (255, 255, 0))
            put(view, 'PICK_WATCH: ' + ' '.join('%.1f' % a for a in PICK_WATCH),
                4, (255, 200, 0))

            cv2.imshow(WINDOW, view)

            now = time.monotonic()
            if now - last_print >= PRINT_INTERVAL:
                last_print = now
                print('[%s] %s | 识别率 %d/%d (%.0f%%)'
                      % ('YES' if found else ' NO', line, hit, total, rate))

            if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                break
    except KeyboardInterrupt:
        pass
    finally:
        scanner.release_resources()
        cv2.destroyAllWindows()
        print('\nPICK_WATCH = %s' % PICK_WATCH)
        print('识别率 %d/%d (%.0f%%)'
              % (hit, total, 100.0 * hit / total if total else 0.0))


if __name__ == '__main__':
    main()
