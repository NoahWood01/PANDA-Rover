import cv2 as cv
import numpy as np
import sys
import time


print(cv.__version__)


input_image = cv.imread("samples/sample_qr_code.png")

qr_detector = cv.QRCodeDetector()

data, points, _ = qr_detector.detectAndDecode(input_image)

if data:
    print(f"QR code data: {data}")
else:
    print("QR Code not detected")