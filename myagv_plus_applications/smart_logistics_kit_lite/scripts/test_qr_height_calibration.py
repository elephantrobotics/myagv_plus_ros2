#!/usr/bin/env python3

import statistics

from smart_logistics_kit_lite.arm_controller import MechArm270Control
from smart_logistics_kit_lite.QRCodeScanner import (
    PICK_HEIGHT_HIGH, PICK_HEIGHT_LOW, VISUAL_LOW_EDGE_MAX, VISUAL_LOW_TVEC_Z_MIN)


ARM_PORT = '/dev/ttyACM0'
ARM_BAUDRATE = 115200
QR_CAMERA = '/dev/video1'
QR_TIMEOUT = 30.0

LOW_LABEL = f'{PICK_HEIGHT_LOW:.0f}'
HIGH_LABEL = f'{PICK_HEIGHT_HIGH:.0f}'


def stats(values):
    if not values:
        return 'n=0'
    return (
        f"n={len(values)} min={min(values):.3f} "
        f"median={statistics.median(values):.3f} max={max(values):.3f}"
    )


def recommend(low_edge, high_edge, low_tvec, high_tvec):
    if not low_edge or not high_edge:
        return
    edge_mid = (statistics.median(low_edge) + statistics.median(high_edge)) / 2.0
    tvec_mid = (statistics.median(low_tvec) + statistics.median(high_tvec)) / 2.0
    print(f"建议 VISUAL_LOW_EDGE_MAX = {edge_mid:.2f}  (当前: {VISUAL_LOW_EDGE_MAX})")
    print(f"建议 VISUAL_LOW_TVEC_Z_MIN = {tvec_mid:.4f}  (当前: {VISUAL_LOW_TVEC_Z_MIN})")
    if max(low_edge) >= min(high_edge):
        print("警告: edge 低/高样本重叠")
    if min(low_tvec) <= max(high_tvec):
        print("警告: tvec_z 低/高样本重叠")


def main():
    arm = MechArm270Control(
        port=ARM_PORT,
        baudrate=ARM_BAUDRATE,
        qr_camera=QR_CAMERA,
        qr_timeout=QR_TIMEOUT,
        qr_show_window=True,
    )
    low_edge, high_edge = [], []
    low_tvec, high_tvec = [], []
    print("edge/tvec_z 高度标定(与生产 QRCodeScanner 同源)。")
    print(f"回车扫一次 → 标注 {LOW_LABEL}(低)/{HIGH_LABEL}(高)/s(跳过),q 结束。")

    try:
        while True:
            cmd = input("\n扫描> ").strip().lower()
            if cmd in {'q', 'quit', 'exit'}:
                break

            arm.move_angles(arm.angle_table['pick_watch'], 50)
            scanner = arm.get_scanner()
            try:
                result = scanner.start_capture()
            except RuntimeError as error:
                print(f"相机打开失败: {error}")
                arm.reset_scanner()
                continue
            if not result or result == -1:
                print("未识别到二维码")
                continue

            edge_med = scanner.last_visual_edge_median
            tvec_z = scanner.last_visual_tvec_z
            if edge_med is None or tvec_z is None:
                print("本帧无 edge/tvec")
                continue
            print(f"edge_med={edge_med:.1f}  tvec_z={tvec_z:.4f}")

            label = input(f"这是 {LOW_LABEL}(低) / {HIGH_LABEL}(高) / s(跳过)? ").strip().lower()
            if label == LOW_LABEL:
                low_edge.append(edge_med)
                low_tvec.append(tvec_z)
            elif label == HIGH_LABEL:
                high_edge.append(edge_med)
                high_tvec.append(tvec_z)
            else:
                print("跳过")
                continue

            print(f"低 {LOW_LABEL} edge: {stats(low_edge)} | tvec: {stats(low_tvec)}")
            print(f"高 {HIGH_LABEL} edge: {stats(high_edge)} | tvec: {stats(high_tvec)}")
            recommend(low_edge, high_edge, low_tvec, high_tvec)
    finally:
        arm.reset_scanner()
        print(f"\n低 {LOW_LABEL} edge: {stats(low_edge)} | tvec: {stats(low_tvec)}")
        print(f"高 {HIGH_LABEL} edge: {stats(high_edge)} | tvec: {stats(high_tvec)}")
        recommend(low_edge, high_edge, low_tvec, high_tvec)


if __name__ == '__main__':
    main()
