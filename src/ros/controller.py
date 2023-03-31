#!/usr/bin/env python

"""
Master controller for the Rover.

must be python 2

ROS nodes need to be added to CMakeLists.txt
under catkin_install_python(PROGRAMS ...
"""

import rospy
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from rospy_tutorials.msg import Floats
from sensor_msgs.msg import Image as msg_Image
from panda_rover.msg import detection_bounds, detection_bound
from rospy.numpy_msg import numpy_msg
import numpy as np
import sys
import os
import math


ROSPY_RATE = 1

# pub = rospy.Publisher('****MOTOR STUFF GOES HERE****', detection_bounds, queue_size=1000)

OBJECT_DETECTION_BOUNDS_TOPIC = "/object_detection_bounds"
DEPTH_IMAGE_TOPIC = "/camera/depth/image_rect_raw"

# **** ALL UNITS EXPRESSED IN MM
OPENING_WIDTH = 381.0

class MasterController:
    def __init__(self):
        self.cv_bridge = CvBridge()
        self.detections_sub = rospy.Subscriber(OBJECT_DETECTION_BOUNDS_TOPIC, detection_bounds, self.data_callback)
        self.depth_image_sub = rospy.Subscriber(DEPTH_IMAGE_TOPIC, msg_Image, self.depth_image_callback)
        

    def data_callback(self, data):
        try:
            detections = data.detections
            # print(detections)
            openings = []
            print(len(detections))
            if len(detections) > 0:
                for detection in detections:
                    if detection.class_label == "opening":
                        openings.append(detection)
                # # TODO make this loop for all found detections
                # x_min, y_min = detections[0].x_min, detections[0].y_min
                # x_max, y_max = detections[0].x_max, detections[0].y_max
                # print("detection corner: ", x_min, y_min, self.get_depth_at_pixel(x_min, y_min))

            # get "front most" and biggest opening
            if len(openings) > 0:
                opening_to_move_to = self.get_opening_by_size(openings)
                x_min, y_min = opening_to_move_to.x_min, opening_to_move_to.y_min
                x_max, y_max = opening_to_move_to.x_max, opening_to_move_to.y_max
                print("opening chosen: ", x_min, y_min, x_max, y_max)
                depths = self.get_depths_for_bounding_box(x_min, y_min, x_max, y_max)
                opening_angle_orientation = self.get_opening_orientation(depths)
                print("depth at corners: ", depths)
                print("angle of opening: ", opening_angle_orientation)
        except Exception as e:
            print(e)


    def depth_image_callback(self, data):
        try:
            self.curr_depth_image = self.cv_bridge.imgmsg_to_cv2(data, data.encoding)
            # pix = (data.width/2, data.height/2)
            # sys.stdout.write('Depth at center(%d, %d): %f(mm)\r' % (pix[0], pix[1], cv_image[pix[1], pix[0]]))    
        except CvBridgeError as e:
            print(e)


    def get_depth_at_pixel(self, x, y):
        # [y, x] because that is how it is stored in cv_bridge
        # check with print(self.get_depth_at_pixel(100, 1000)) and print(self.get_depth_at_pixel(1000, 100))
        return self.curr_depth_image[y, x]


    def get_depths_for_bounding_box(self, x_min, y_min, x_max, y_max):
        depths = []
        depths.append(self.get_depth_at_pixel(x_min, y_min))
        depths.append(self.get_depth_at_pixel(x_min, y_max))
        depths.append(self.get_depth_at_pixel(x_max, y_min))
        depths.append(self.get_depth_at_pixel(x_max, y_max))
        return depths


    def get_opening_orientation(self, depths):
        d1 = min(depths[0], depths[1])
        d2 = min(depths[2], depths[3])

        numerator = d1 ** 2 - OPENING_WIDTH ** 2 - d2 ** 2
        denominator = -2 * OPENING_WIDTH * d2
        print(d1,d2,numerator,denominator , numerator/denominator)
        return math.asin( numerator / denominator )


    def get_opening_by_size(self, openings):
        """
        gets the opening that is the biggest, theoretically should be the frontmost

        TODO deal with multiple openings from more than 1 box.
        """
        curr_chosen_opening = openings[0]
        for opening in openings:
            x_min, x_max = opening.x_min, opening.x_max
            y_min, y_max = opening.y_min, opening.y_max

            # must completely overlap
            if (
                x_min <= curr_chosen_opening.x_min and
                x_max >= curr_chosen_opening.x_max and
                y_min <= curr_chosen_opening.y_min and
                y_max >= curr_chosen_opening.y_max
            ):
                curr_chosen_opening = opening
        
        return curr_chosen_opening



if __name__ == '__main__':
    rospy.init_node("controller")
    rate = rospy.Rate(ROSPY_RATE)
    controller = MasterController()
    print('rover controller started')
    while not rospy.is_shutdown():
        # pub.publish(things)
        rate.sleep()
