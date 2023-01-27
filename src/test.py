import cv2 as cv
import numpy as np
import sys
import time

from cv.cv import detect_qr_code, template_match
import ros


# print(cv.__version__)

test_img = 'samples/box_qr_1.png'
detect_qr_code(test_img)

test_temp_image = "samples/test_image0.png"
test_temp = "templates/temp_blue_open.png"
template_match(test_temp_image, test_temp)