import logging
import os

import cv2
import numpy as np
from paddleocr import PaddleOCR
from PIL import Image, ImageDraw, ImageFont
from ament_index_python.packages import get_package_share_directory

logging.getLogger('ppocr').setLevel(logging.WARNING)


class OCRVideoCapture:
    def __init__(self, font_size=40):
        pkg_dir = get_package_share_directory('smart_logistics_kit')
        font_path = os.path.join(pkg_dir, 'SIMFANG.TTF')
        self.ocr = PaddleOCR(use_angle_cls=True, lang='ch')
        self.font = ImageFont.truetype(font_path, font_size)
        self.text_color = (0, 255, 0)

    def recognize(self, frame):
        texts = []
        result = self.ocr.ocr(frame, cls=True)
        if not result or result == [None]:
            return frame, texts

        for line in result:
            if not line:
                continue
            for word_info in line:
                box = np.array(word_info[0]).astype(np.int32)
                cv2.polylines(frame, [box], isClosed=True, color=(0, 255, 0), thickness=2)

        pil_image = Image.fromarray(frame)
        draw = ImageDraw.Draw(pil_image)
        for line in result:
            if not line:
                continue
            for word_info in line:
                text = word_info[1][0]
                texts.append(text)
                box = word_info[0]
                center_x = (box[0][0] + box[2][0]) / 2
                center_y = (box[0][1] + box[2][1]) / 2
                draw.text((center_x, center_y), text, font=self.font, fill=self.text_color)

        return np.array(pil_image), texts
