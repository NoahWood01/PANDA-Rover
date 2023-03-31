#!/usr/bin/env python3

"""
Subscriber to /image_data topic

#!/usr/bin/env python3 is needed for tensorflow

Takes input from image_data representing the rgb image from the realsense

ROS nodes need to be added to CMakeLists.txt
under catkin_install_python(PROGRAMS ...
"""

import rospy
from std_msgs.msg import String
from rospy_tutorials.msg import Floats
from panda_rover.msg import detection_bounds, detection_bound
# from sensor_msgs.msg import Image as msg_Image
from rospy.numpy_msg import numpy_msg
# from cv_bridge import CvBridge
import numpy as np
import sys
import os
import cv2
from tflite_runtime.interpreter import Interpreter

MODEL_PATH = "../catkin_ws/src/PANDA-Rover/src/ros/custom_model_lite/detect.tflite"

min_conf_threshold = 0.5
ROSPY_RATE = 1

pub = rospy.Publisher('/object_detection_bounds', detection_bounds, queue_size=1000)

# debug_cv_image_pub = rospy.Publisher('/object_detection_bounding_boxes', msg_Image, queue_size=1000)

class ObjectDetectionListener:
    def __init__(self, topic):
        self.topic = topic
        self.detections = []
        self.cv_image = None
        # self.cv_bridge = CvBridge()
        self.sub = rospy.Subscriber(topic, numpy_msg(Floats), self.data_callback)
        # f = open(MODEL_PATH)
        # print(os.getcwd())
        # print(f.read())
        self.tf_interpreter = Interpreter(MODEL_PATH)
        self.tf_interpreter.allocate_tensors()
        self.input_details = self.tf_interpreter.get_input_details()
        self.output_details = self.tf_interpreter.get_output_details()
        self.height = self.input_details[0]['shape'][1]
        self.width = self.input_details[0]['shape'][2]
        self.labels = ['box', 'opening']
        self.input_mean = 127.5
        self.input_std = 127.5

        self.floating_model = (self.input_details[0]['dtype'] == np.float32)

    def data_callback(self, data):
        try:
            outname = self.output_details[0]['name']
            if ('StatefulPartitionedCall' in outname): # This is a TF2 model
                boxes_idx, classes_idx, scores_idx = 1, 3, 0
            else: # This is a TF1 model
                boxes_idx, classes_idx, scores_idx = 0, 1, 2
            cv_image = self.reshape_image(data)
            # ensure same size as model
            image_resized = cv2.resize(cv_image, (self.width, self.height))
            input_data = np.expand_dims(image_resized, axis=0)

            if self.floating_model:
                input_data = (np.float32(input_data) - self.input_mean) / self.input_std

            self.tf_interpreter.set_tensor(self.input_details[0]['index'],input_data)
            self.tf_interpreter.invoke()

            imH, imW, _ = cv_image.shape 
            # Retrieve detection results
            boxes = self.tf_interpreter.get_tensor(self.output_details[boxes_idx]['index'])[0] # Bounding box coordinates of detected objects
            classes = self.tf_interpreter.get_tensor(self.output_details[classes_idx]['index'])[0] # Class index of detected objects
            scores = self.tf_interpreter.get_tensor(self.output_details[scores_idx]['index'])[0] # Confidence of detected objects
            detections = []
            # print(boxes)
            # print(classes)
            # print(scores)
            # Loop over all detections and draw detection box if confidence is above minimum threshold
            for i in range(len(scores)):
                if ((scores[i] > min_conf_threshold) and (scores[i] <= 1.0)):

                    # Get bounding box coordinates and draw box
                    # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
                    ymin = int(max(1,(boxes[i][0] * imH)))
                    xmin = int(max(1,(boxes[i][1] * imW)))
                    ymax = int(min(imH,(boxes[i][2] * imH)))
                    xmax = int(min(imW,(boxes[i][3] * imW)))
                    # print(xmin, xmax, ymin, ymax)
                    cv2.rectangle(cv_image, (xmin,ymin), (xmax,ymax), (10, 255, 0), 2)

                    # Draw label
                    object_name = self.labels[int(classes[i])] # Look up object name from "labels" array using class index
                    label = '%s: %d%%' % (object_name, int(scores[i]*100)) # Example: 'person: 72%'
                    labelSize, baseLine = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2) # Get font size
                    label_ymin = max(ymin, labelSize[1] + 10) # Make sure not to draw label too close to top of window
                    cv2.rectangle(cv_image, (xmin, label_ymin-labelSize[1]-10), (xmin+labelSize[0], label_ymin+baseLine-10), (255, 255, 255), cv2.FILLED) # Draw white box to put label text in
                    cv2.putText(cv_image, label, (xmin, label_ymin-7), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 2) # Draw label text
                    msg = detection_bound()
                    msg.class_label = object_name
                    msg.confidence = scores[i]
                    msg.x_min = xmin
                    msg.y_min = ymin
                    msg.x_max = xmax
                    msg.y_max = ymax
                    detections.append(msg)

            # cv2.imwrite("../catkin_ws/src/PANDA-Rover/model_detect_res.png", cv_image)
            # self.cv_image = self.cv_bridge.cv2_to_imgmsg(cv_image, "passthrough")
            # sys.stdout.write(f"{len(detections)} were detected\n")
            # sys.stdout.write(f"{detections} were detected\n")
            # sys.stdout.flush()
            self.detections = detections
        except Exception as e:
            print(e)
            return

    def reshape_image(self, data):
        cv_image = np.reshape(data.data, (-1, 640, 3))
        return cv_image

    def detect_image(self):
        pass

if __name__ == '__main__':
    rospy.init_node("object_detction_model")
    rate = rospy.Rate(ROSPY_RATE)
    topic = '/image_data'
    listener = ObjectDetectionListener(topic)
    print('object_detction_model subscriber listening!')
    while not rospy.is_shutdown():
        pub.publish(listener.detections)
        # if listener.cv_image != None:
        #     debug_cv_image_pub.publish(listener.cv_image)
        rate.sleep()