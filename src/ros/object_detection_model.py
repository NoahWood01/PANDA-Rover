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
from rospy.numpy_msg import numpy_msg
import numpy as np
import sys
import os
import cv2
from tflite_runtime.interpreter import Interpreter


MODEL_PATH = "src/ros/custom_model_lite/detect.tflite"

min_conf_threshold = 0.5

class ObjectDetectionListener:
    def __init__(self, topic):
        self.topic = topic
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

                    detections.append([object_name, scores[i], xmin, ymin, xmax, ymax])

            cv2.imwrite("model_detect_res.png", cv_image)
            sys.stdout.write(f"{len(detections)} were detected")
            sys.stdout.flush()
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
    topic = '/image_data'
    listener = ObjectDetectionListener(topic)
    print('object_detction_model subscriber listening!')
    rospy.spin()