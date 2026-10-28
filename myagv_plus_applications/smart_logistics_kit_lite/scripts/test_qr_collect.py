#!/usr/bin/env python3

import re
import sys

import cv2
import numpy as np
import yaml
from pyzbar.pyzbar import decode


CAMERA_DEVICE = '/dev/video1'
OUTPUT_FILE = 'qr_texts.yaml'
WINDOW = 'QR Collect'


def extract_city(qr_data):
    text = qr_data.strip()
    match = re.search(r'地址[:：]\s*(.+?市)', text)
    if match:
        city = match.group(1).strip()
        return city.split('省')[-1].strip() if '省' in city else city
    match = re.search(r'(.+?City)', text)
    if match:
        city = match.group(1).strip()
        return city.split('Province')[-1].strip() if 'Province' in city else city
    return text


def save(found):
    data = {'qr_texts': [{'text': t, 'city': c} for t, c in found.items()]}
    with open(OUTPUT_FILE, 'w', encoding='utf-8') as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)


def main():
    device = sys.argv[1] if len(sys.argv) > 1 else CAMERA_DEVICE
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        cap.release()
        cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        print(f'无法打开相机 {device}')
        return
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    print(f'相机 {device}，按 q 或 ESC 退出，结果写入 {OUTPUT_FILE}\n')
    found = {}
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print('读帧失败')
                break

            for obj in decode(frame):
                text = obj.data.decode('utf-8', errors='ignore').strip()
                if not text:
                    continue
                if text not in found:
                    found[text] = extract_city(text)
                    print(f'[{len(found):02d}] {text}   -> city: {found[text]}')
                    save(found)
                pts = obj.polygon
                if len(pts) == 4:
                    cv2.polylines(frame, [np.array([[p.x, p.y] for p in pts], dtype='int32')],
                                  True, (0, 255, 0), 2)

            cv2.putText(frame, f'unique={len(found)}', (10, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2, cv2.LINE_AA)
            cv2.imshow(WINDOW, frame)
            if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
                break
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print(f'\n共收集 {len(found)} 条，已写入 {OUTPUT_FILE}')


if __name__ == '__main__':
    main()
