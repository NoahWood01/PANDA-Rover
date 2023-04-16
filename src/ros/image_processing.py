#!/usr/bin/env python

"""
Subscriber to /image_data topic

Takes input from image_data representing the rgb image from the realsense

ROS nodes need to be added to CMakeLists.txt
under catkin_install_python(PROGRAMS ...
"""

import rospy
from std_msgs.msg import String
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
import numpy as np
import sys
import os
import cv2


class ImageDataListener:
    def __init__(self, topic):
        self.topic = topic
        self.sub = rospy.Subscriber(topic, numpy_msg(Floats), self.data_callback)

    def data_callback(self, data):
        try:
            cv_image = self.reshape_image(data)
            # sys.stdout.write("%s Image shape: %s \r" % (self.topic, str(cv_image.shape)))
            # sys.stdout.flush()
        except Exception as e:
            print(e)
            return

    def reshape_image(self, data):
        cv_image = np.reshape(data.data, (-1, 640, 3))
        return cv_image

if __name__ == '__main__':
    rospy.init_node("cv_image_processor")
    topic = '/image_data'
    listener = ImageDataListener(topic)
    print('cv_image_processor subscriber listening!')
    rospy.spin()