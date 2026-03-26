import cv2
from pyzbar.pyzbar import decode
import numpy as np
import re
from PIL import Image, ImageDraw, ImageFont
import time
from ament_index_python.packages import get_package_share_directory
import os

class QRCodeScanner:
    def __init__(self,videoid="/dev/video1", camera_index=1, font_path="./SIMFANG.TTF", font_size=25):
        self.camera_index = camera_index
        share_dir = get_package_share_directory('smart_logistics_kit_lite')
        self.font_path = os.path.join(share_dir, 'resource', 'SIMFANG.TTF')
        self.font_size = font_size
        self.font = ImageFont.truetype(self.font_path, self.font_size)
        self.time_out=60
        self.cap = cv2.VideoCapture(videoid)
        self.text_color = (0, 255, 0)
        self.camera_matrix = np.array([
            [827.29511682,  0.,             368.87666292],
            [0.,            824.88958537,   262.03016541],
            [0.,            0.,             1.          ]])

        self.marker_size=0.027
        self.dist_coeffs = np.array(([[0.21780081, -0.56324781, 0.01165061, 0.01845253, -1.0631406]]))
        
        self.marker_points = np.array([[-self.marker_size/2, self.marker_size/2, 0], [self.marker_size/2, self.marker_size/2, 0],
                                        [self.marker_size/2, -self.marker_size/2, 0], [-self.marker_size/2, -self.marker_size/2, 0]], dtype=np.float32)
       
        if not self.cap.isOpened():
            raise Exception("Failed to open camera")

    def scan_qrcode_from_camera(self,raw_frame):

        decoded_objects = decode(raw_frame)
        city = "Unknown City"
        if decoded_objects:
            
            for obj in decoded_objects:
                qr_data = obj.data.decode("utf-8")
                # print(f"{i} QR Code Data: {qr_data}")
                match = re.search(r'(.+?City)',  qr_data)
                if match:
                    city = match.group(1)
                    if "Province" in city:
                        parts =city.split("Province")
                        city=parts[-1]
                else:
                    print("City information not found")
                points = obj.polygon
                if len(points) == 4:  # 

                    pts = np.array(points, dtype=np.int32)
                    
                    corners=(np.array([[pts[2],pts[1],pts[0],pts[3]]], dtype=np.float32))
     
                    cv2.polylines(raw_frame, [pts], isClosed=True, color=(255, 0, 0), thickness=2)
                    _,rvec, tvec, = cv2.solvePnP(self.marker_points,np.float32(pts), self.camera_matrix, self.dist_coeffs)
                    xy=[round(x * 1000, 2) for x in tvec.flatten()]
                    tvec=tvec.T.reshape(1,1,3)
                    # print("xy",xy)
                    
                    x, y, w, h = cv2.boundingRect(pts)
                    pil_image = Image.fromarray(raw_frame)
                    draw = ImageDraw.Draw(pil_image)
                    bbox = draw.textbbox((x, y), qr_data, font=self.font)
                    text_width = bbox[2] - bbox[0]
                    text_height = bbox[3] - bbox[1]

                    text_x = x + (w - text_width) // 2
                    text_y = y + (h - text_height) // 2

                    draw.text((text_x, text_y), qr_data, font=self.font, fill= self.text_color)
                    qr_frame = np.array(pil_image)
                   
                    return [qr_frame,city,tvec]

    def start_capture(self):
        while True:
            self.start_time=time.time()
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to read video stream")
                break
            result=self.scan_qrcode_from_camera(frame)
            
            if result:
                cv2.imshow("QR Code Scanner", result[0])
                cv2.waitKey(1500)
                cv2.destroyAllWindows()
                return result[1:]
            else:
                cv2.imshow("QR Code Scanner",frame)
            
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    cv2.destroyAllWindows()
                    break
            # print(time.time()-self.start_time)
            if time.time()-self.start_time>self.time_out:
                print("60s recognition timeout")
                cv2.destroyAllWindows()
                return -1

    def release_resources(self):
        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    scanner = QRCodeScanner()
    for i in range(1):
        print(scanner.start_capture())
   