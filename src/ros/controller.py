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
from sensor_msgs.msg import Image as msg_Image, LaserScan
from panda_rover.msg import detection_bounds, detection_bound, lidar_msg, lidar_msgs
from rospy.numpy_msg import numpy_msg
import sys
import os
import math
from enum import Enum

# Interfacing with Drone
#from fly_tello import FlyTello

my_tellos = list()


from test_movement import MovementCalculator
from robot_mode.approach_mode import approach

ROSPY_RATE = 1

# pub = rospy.Publisher('****MOTOR STUFF GOES HERE****', detection_bounds, queue_size=1000)

OBJECT_DETECTION_BOUNDS_TOPIC = "/object_detection_bounds"
DEPTH_IMAGE_TOPIC = "/camera/depth/image_rect_raw"
LIDAR_BOX_ANGLES_TOPIC = "/get_angles"
DRONE_QR_READER_SUB = "zbar/barcode_drone"
DRONE_COMMANDS_TOPIC = '/drone_commands'
LIDAR_SCAN_TOPIC = '/front_lidar_scan'

# **** ALL UNITS EXPRESSED IN MM
OPENING_WIDTH = 381.0

CV_BOX_DEPTH_LIMIT = 5000


DRONE_MOVEMENT_SPEED = 5000
DRONE_ROTATION_SPEED = 5000


ROVER_HOME_QR_CODE = "PANDA"


# Subscriber for the QR code
def qr_callback(self,data):
	try:
		rospy.loginfo(rospy.get_caller_id() + "QR: %s", data.data)
	except Exception as e:
		print(e)

def qr_listener():
	rospy.init_node("qr_listener", anonymous=True)
	try:
		rospy.Subscriber("/barcode", String, qr_callback)
		rospy.spin()
	except rospy.ROSInterruptException:
		pass



class RobotMode(Enum):
	# use lidar to find boxes and prepare to send drone
	search = "search"
	# check a found box with drone if it is next box
	# if correct goto approach else search
	send_drone = "send_drone"
	# move to located box
	approach = "approach"
	# enter and move through box scanning next qr code
	enter = "enter"
	# attempt touch and go
	touch_and_go = "touch_and_go"
	# repeat

	# end of course
	done = "done"

	# something goes wrong
	fucked = "fucked"


class MasterController:
	def __init__(self, no_drone=False):
		self.cv_bridge = CvBridge()
		self.mode = RobotMode('search')

		if not no_drone:
			pass
			# Stage 1 (Search) Subscription

			#self.drone_qr_reader_sub = rospy.Subscriber(DRONE_QR_READER_SUB, String, self.drone_qr_reader_callback)
		
		self.drone_command_pub = rospy.Publisher(DRONE_COMMANDS_TOPIC, String, queue_size=1000)

		# TODO: Add IMU topics
		# self.drone_imu_status = 0
		# self.rover_imu_status = 0

		# Stage 1 Variables
		self.drone_qr_code = ""
		self.next_box_id = ""

		# Maintaining box order
		self.current_box = ""
		self.previous_boxes = []
		# self.next_box_id_pub = rospy.Publisher(NEXT_BOX_ID_TOPIC, String, queue_size=1000)
		# self.correct_box_sub = rospy.Subscrbier(CORRECT_BOX_TOPIC, String, self.correct_box_callback)

		self.lidar_boxes = []
		self.front_lidar_scan = []

		self.detections_sub = rospy.Subscriber(OBJECT_DETECTION_BOUNDS_TOPIC, detection_bounds, self.data_callback)
		self.depth_image_sub = rospy.Subscriber(DEPTH_IMAGE_TOPIC, msg_Image, self.depth_image_callback)
		


		self.opening_angle_orientation = None 
		self.curr_depth_image = None

		self.movement_calculator = MovementCalculator()

		self.lidar_box_angles_sub = rospy.Subscriber(LIDAR_BOX_ANGLES_TOPIC, lidar_msgs, self.lidar_box_angles_callback)
		self.lidar_scan_sub = rospy.Subscriber(LIDAR_SCAN_TOPIC, Floats, self.lidar_scan_callback)



	def correct_box_callback(self, data):
		# Process targeted box location 
		x = 1

	def lidar_box_angles_callback(self, boxes):
		self.lidar_boxes = boxes.box_estimates
		# print("boxes %s" % boxes)
		# Process targeted box location 
		# if self.mode == RobotMode('search'):
		# 	if len(boxes) == 0:
		# 		self.mode = RobotMode('fucked')
		# 		return
		# 	for box in boxes:
		# 		self.mode = RobotMode('send_drone')
		# 		if self.send_drone_to_box(box) == True:
		self.mode = RobotMode('approach')
		# 			return
		# 	self.mode = RobotMode('fucked')


	def lidar_scan_callback(self, scan_data):
		front_lidar_scan = scan_data.data
		self.front_lidar_scan = front_lidar_scan


	def drone_qr_reader_callback(self, id):
		self.drone_qr_code = id

	def data_callback(self, data):
		try:
			detections = data.detections
			#print(detections)
			openings = []
			#print(len(detections))
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
				#print("opening chosen: ", x_min, y_min, x_max, y_max)
				depths = self.get_depths_for_bounding_box(x_min, y_min, x_max, y_max)
				print(depths)
				opening_angle_orientation = self.get_opening_orientation(depths)
				if type(opening_angle_orientation) == float:
					self.opening_angle_orientation = opening_angle_orientation
				#print("depth at corners: ", depths)
				#print("angle of opening: ", opening_angle_orientation)
		except Exception as e:
			print(e)


	def depth_image_callback(self, data):
		try:
			self.curr_depth_image = self.cv_bridge.imgmsg_to_cv2(data, data.encoding)
			# pix = (data.width/2, data.height/2)
			# sys.stdout.write('Depth at center(%d, %d): %f(mm)\r' % (pix[0], pix[1], cv_image[pix[1], pix[0]]))    
		except CvBridgeError as e:
			print(e)


	def send_drone_to_box(self, box):
		found_box = False
        
		# drone.takeoff()

		# Get IMU data from rover and drone to line up drone path
		# realign_drone()

		# drone.clockwise(DRONE_ROTATION_SPEED)
		# sleep( scanned_box.theta / DRONE_ROTATION_SPEED )
		# drone.counter_clockwise(0)

		# drone.forward(DRONE_MOVEMENT_SPEED)
		# sleep( scanned_box.distance / DRONE_MOVEMENT_SPEED )
		# drone.backward(0)
        
		# Process QR Code
		found_box = qr_code_id == self.get_drone_qr_code()

		# Rebase Drone
		# drone.backward(DRONE_MOVEMENT_SPEED)
		# sleep( scanned_box.distance / DRONE_MOVEMENT_SPEED )
		# drone.forward(0)

		# # Realign drone
		# drone.counter_clockwise(DRONE_ROTATION_SPEED)
		# sleep( scanned_box.theta / DRONE_ROTATION_SPEED )
		# drone.clockwise(0)

		# # Process PANDA Rover QR code
		# while qr_code_id != ROVER_HOME_QR_CODE:
		#     # Arbitruary movement?
		#     break
		# drone.land()
		# sleep(3)
		return found_box


	def get_drone_qr_code(self):
		return self.drone_qr_code

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
		d1 = depths[1]
		d2 = depths[3]

		if d1 == 0 or d2 == 0 or d1 > CV_BOX_DEPTH_LIMIT or d2 > CV_BOX_DEPTH_LIMIT:
			return "BAD INPUT"

		numerator = d1 ** 2 - OPENING_WIDTH ** 2 - d2 ** 2
		denominator = -2 * OPENING_WIDTH * d2
		# print(d1,d2,numerator,denominator , numerator/denominator)
		angle = math.asin( numerator / denominator )
		print("angle %s" % angle)
		return angle


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


def search(controller):
	pass


def send_drone(controller):
	pass


#def approach(controller):
	#"""
	#assumption: already facing the box.
	#"""
	#print("controller %s" % controller.opening_angle_orientation)
	#pass


def enter(controller):
	pass



if __name__ == '__main__':
	no_drone = False
	if sys.argv[-1] == "no_drone":
		no_drone = True
	rospy.init_node("controller")
	rate = rospy.Rate(ROSPY_RATE)
	controller = MasterController(no_drone=no_drone)
	print('rover controller started')

	try:
		pass
		# TODO move this outside of controller.py 
		# cant init 2 nodes in same file
		#qr_listener()
	except rospy.ROSInterruptException:
		rospy.logerr("Failed to init QR listener node.")


	# Wait for movement topics to initalize
	# while True:
	# 	if controller.movement_calculator.pub.get_num_connections() > 0:
	# 		break

	#controller.movement_calculator.move_forward()
	#controller.movement_calculator.rotate_counterclockwise(90)

	print("test")
	# reset	
	controller.mode = RobotMode.approach
	while not rospy.is_shutdown() and controller.mode != RobotMode.done:
		print("Looped")
		# pub.publish(things)
		# DO move stuff 
		if controller.mode == RobotMode.approach:
			approach(controller)
			break
		rate.sleep()
