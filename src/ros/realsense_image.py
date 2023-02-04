#!/usr/bin/env python

"""
Subscriber to realsense camera.

Takes input from realsense image view and gives image data as cv image

ROS nodes need to be added to CMakeLists.txt
under catkin_install_python(PROGRAMS ...
"""

import rospy
from sensor_msgs.msg import Image as msg_Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import sys
import os

class ImageListener:
    def __init__(self, topic):
        self.topic = topic
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic, msg_Image, self.image_callback)

    def image_callback(self, data):
        try:
            # this image is represented as np.ndarray of shape (resolution_x, resolution_y, 3)
            # where 3 is RGB
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            pix = (data.width/2, data.height/2)
            sys.stdout.write('%s: cv image data at center(%d, %d): %s\r' % (self.topic, pix[0], pix[1], np.array_str(cv_image[pix[1], pix[0]])))
            sys.stdout.flush()
        except CvBridgeError as e:
            print(e)
            return


if __name__ == '__main__':
    rospy.init_node("depth_image_processor")
    topic = '/camera/color/image_raw' 
    listener = ImageListener(topic)
    print('realsense subscriber listening!')
    rospy.spin()