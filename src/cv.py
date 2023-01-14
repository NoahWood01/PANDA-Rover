import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

def detect_qr_code(image):
    """
    Finds data from image containing qr code.
    """
    cv_input_image = cv.imread(image)

    qr_detector = cv.QRCodeDetector()

    data, points, _ = qr_detector.detectAndDecode(cv_input_image)

    if data:
        print(f"QR code data: {data}")
    else:
        print("QR Code not detected")

def template_match(input_image, source_template, method=cv.TM_CCOEFF_NORMED):
    """
    Uses template matching to display a resulting image 'res.png' with 
    bounding rectangles on matched locations
    https://docs.opencv.org/4.x/d4/dc6/tutorial_py_template_matching.html
    """
    # confidence threshold for cv detection
    THRESHOLD = 0.5

    img_rgb = cv.imread(input_image)
    img_gray = cv.cvtColor(img_rgb, cv.COLOR_BGR2GRAY)
    template = cv.imread(source_template, 0)

    w, h = template.shape[::-1]
    res = cv.matchTemplate(img_gray, template, method)
    loc = np.where( res >= THRESHOLD)
    # each point found pt
    for pt in zip(*loc[::-1]):
        cv.rectangle(img_rgb, pt, (pt[0] + w, pt[1] + h), (0, 255, 0), 2)
    cv.imwrite('res.png', img_rgb)