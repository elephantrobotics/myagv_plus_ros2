import cv2
from pyzbar.pyzbar import decode
import numpy as np
import re
from PIL import Image, ImageDraw, ImageFont
import time
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
import os


# 低抓 Z=50：edge_med 小且 tvec_z 大;否则默认高抓 Z=92。
# 阈值取两类中点以留足余量(标定: LOW edge≈64.1/tvec≈0.348, HIGH edge≈79.8/tvec≈0.278)。
# 用 test_qr_height_calibration.py 采样后取中点更新这两个值。
VISUAL_LOW_EDGE_MAX = 71.93
VISUAL_LOW_TVEC_Z_MIN = 0.3129


class QRCodeScanner:
    def __init__(
        self,
        camera_index=1,
        video_device="/dev/video1",
        font_path=None,
        font_size=25,
        timeout=60.0,
        show_window=True,
    ):
        self.camera_index = camera_index
        self.video_device = video_device if video_device is not None else camera_index
        if font_path is None:
            font_path = self.get_default_font_path()
        self.font_path = font_path
        self.font_size = font_size
        try:
            self.font = ImageFont.truetype(self.font_path, self.font_size)
        except OSError:
            self.font = ImageFont.load_default()
        self.time_out = float(timeout)
        self.show_window = bool(show_window)
        use_v4l2 = isinstance(self.video_device, str) and self.video_device.startswith("/dev/video")
        if use_v4l2:
            self.cap = cv2.VideoCapture(self.video_device, cv2.CAP_V4L2)
        else:
            self.cap = cv2.VideoCapture(self.video_device)
        if use_v4l2 and not self.cap.isOpened():
            self.cap.release()
            self.cap = cv2.VideoCapture(self.video_device)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.text_color = (0, 255, 0)
        self.last_pick_height = None
        self.last_visual_level = None
        self.last_visual_reason = None
        self.last_visual_edge_median = None
        self.last_visual_tvec_z = None
        self.camera_matrix = np.array([
            [827.29511682, 0., 368.87666292],
            [0.,  824.88958537, 262.03016541],
            [0., 0., 1.]])

        self.marker_size = 0.027
        self.dist_coeffs = np.array(([[0.21780081, -0.56324781, 0.01165061,   0.01845253,
             -1.0631406]]))
        
        self.marker_points = np.array([[-self.marker_size/2, self.marker_size/2, 0], [self.marker_size/2, self.marker_size/2, 0],
                                        [self.marker_size/2, -self.marker_size/2, 0], [-self.marker_size/2, -self.marker_size/2, 0]], dtype=np.float32)
       
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera: {self.video_device}")

    @staticmethod
    def get_default_font_path():
        try:
            pkg_dir = get_package_share_directory('smart_logistics_kit')
            return os.path.join(pkg_dir, 'SIMFANG.TTF')
        except PackageNotFoundError:
            return os.path.join(os.path.dirname(os.path.dirname(__file__)), 'resource', 'SIMFANG.TTF')

    @staticmethod
    def extract_city(qr_data):
        text = qr_data.strip()
        match = re.search(r'地址[:：]\s*(.+?市)', text)
        if match:
            city = match.group(1).strip()
            if "省" in city:
                city = city.split("省")[-1].strip()
            return city

        match = re.search(r'(.+?City)', text)
        if match:
            city = match.group(1).strip()
            if "Province" in city:
                city = city.split("Province")[-1].strip()
            return city

        return text

    @staticmethod
    def destroy_windows():
        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass

    def clear_visual_height(self):
        self.last_pick_height = None
        self.last_visual_level = None
        self.last_visual_reason = None
        self.last_visual_edge_median = None
        self.last_visual_tvec_z = None

    def update_visual_height(self, pts, tvec):
        lengths = [
            float(np.linalg.norm(pts[(index + 1) % 4] - pts[index]))
            for index in range(4)
        ]
        edge_median = float(np.median(lengths))
        tvec_z = float(np.reshape(tvec, 3)[2])

        self.last_visual_edge_median = edge_median
        self.last_visual_tvec_z = tvec_z

        low_match = edge_median <= VISUAL_LOW_EDGE_MAX and tvec_z >= VISUAL_LOW_TVEC_Z_MIN

        if low_match:
            self.last_pick_height = 50.0
            self.last_visual_level = "low"
            self.last_visual_reason = "edge_small_z_large"
        else:
            self.last_pick_height = 92.0
            self.last_visual_level = "high_default"
            self.last_visual_reason = "not_low_default_high"

        print(
            f"[qr-visual] level={self.last_visual_level} "
            f"edge_med={edge_median:.1f} tvec_z={tvec_z:.4f} "
            f"reason={self.last_visual_reason}")

    def flush_frames(self, count=8):
        for _ in range(count):
            if self.cap is None or not self.cap.isOpened():
                return
            self.cap.grab()
            time.sleep(0.01)

    def scan_qrcode_from_camera(self, raw_frame):
        decoded_objects = decode(raw_frame)
        if decoded_objects:
            for obj in decoded_objects:
                qr_data = obj.data.decode("utf-8", errors="ignore")
                city = self.extract_city(qr_data)
                points = obj.polygon
                if len(points) == 4:
                    pts = np.array(points, dtype=np.int32)
     
                    cv2.polylines(raw_frame, [pts], isClosed=True, color=(255, 0, 0), thickness=2)
                    _, _, tvec, = cv2.solvePnP(self.marker_points, np.float32(pts), self.camera_matrix, self.dist_coeffs)
                    tvec = tvec.T.reshape(1, 1, 3)
                    self.update_visual_height(pts, tvec)
                    
                    x, y, w, h = cv2.boundingRect(pts)
                    pil_image = Image.fromarray(raw_frame)
                    draw = ImageDraw.Draw(pil_image)
                    bbox = draw.textbbox((x, y), qr_data, font=self.font)
                    text_width = bbox[2] - bbox[0]
                    text_height = bbox[3] - bbox[1]

                    text_x = x + (w - text_width) // 2
                    text_y = y + (h - text_height) // 2

                    draw.text((text_x, text_y), qr_data, font=self.font, fill=self.text_color)
                    qr_frame = np.array(pil_image)
                   
                    return [qr_frame, city, tvec]
                return [raw_frame, city, None]

    def start_capture(self):
        if self.cap is None or not self.cap.isOpened():
            raise RuntimeError(f"Cannot open camera: {self.video_device}")
        self.clear_visual_height()
        self.flush_frames()
        start_time = time.time()
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("Cannot read camera frame.")
                break
            result = self.scan_qrcode_from_camera(frame)
            
            if result:
                if self.show_window:
                    cv2.imshow("QR Code Scanner", result[0])
                    cv2.waitKey(1500)
                    self.destroy_windows()
                return result[1:]
            elif self.show_window:
                cv2.imshow("QR Code Scanner", frame)
            
            if self.show_window and cv2.waitKey(1) & 0xFF == ord('q'):
                self.destroy_windows()
                break
            
            if time.time() - start_time > self.time_out:
                print(f"QR scan timeout after {self.time_out:.1f} s.")
                self.destroy_windows()
                return -1

    def release_resources(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None
        self.destroy_windows()


if __name__ == "__main__":
    scanner = QRCodeScanner()
    try:
        print(scanner.start_capture())
    finally:
        scanner.release_resources()
