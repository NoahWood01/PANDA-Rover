#!/usr/bin/env python

"""
Master controller for the Rover.

must be python 2

ROS nodes need to be added to CMakeLists.txt
under catkin_install_python(PROGRAMS ...
"""

import rospy
from time import sleep
from std_msgs.msg import String
from sensor_msgs.msg import Imu


ROSPY_RATE = 1


class MovementCalculator(): 
	def __init__(self):
		self.rate = rospy.Rate(ROSPY_RATE)
		self.sub = rospy.Subscriber("/imu/rpy", String, self.imu_callback)
		self.pub = rospy.Publisher('/motor_input_translate', String, queue_size=1000)
		self.angle = 0
		# expressed in degrees
		self.angle_threshold = 1
		self.ROTATE_TIME_IN_S = 0.1
		self.sleep_offset = 9

	def imu_callback(self, imu_msg):
		try:
			self.angle = float(imu_msg.data)
		except Exception as e:
			print(e)


	def get_goal_angle_offset(self, theta):
		"""
		Applies offset formula
		"""
		return theta/18
		#return theta
		
	def rotate_clockwise(self, theta):
		"""
		Angles are based on /imu/rpy which is in degrees
		"""
		# print("start angle", self.angle)
		start_angle = self.angle
		goal_angle = start_angle - theta
		#angle_diff = start_angle - self.angle
		#goal_angle = start_angle + theta
		if goal_angle < -180:
			goal_angle = 180 + (goal_angle + 180) + self.get_goal_angle_offset(theta)

		while not (self.angle > goal_angle-3 and self.angle < goal_angle + 3):
			# print(self.angle)
			msg = "ro,1,1,0.2,%s" % str(self.ROTATE_TIME_IN_S)
			self.pub.publish(msg)
			sleep(self.ROTATE_TIME_IN_S+0.1)
		#angle_diff = start_angle - self.angle
		# print("end angle", self.angle)

	def rotate_counterclockwise(self, theta):
		"""
		Angles are based on /imu/rpy which is in degrees
		"""
		# print("start angle", self.angle)
		start_angle = self.angle
		
		goal_angle = start_angle + theta - self.get_goal_angle_offset(theta)
		#angle_diff = start_angle - self.angle
		#goal_angle = start_angle + theta
		if goal_angle > 180:
			goal_angle = -180 + (goal_angle - 180)

		while not (self.angle > goal_angle-3 and self.angle < goal_angle + 3):
			# print(self.angle)
			msg = "ro,-1,-1,0.2,%s" % str(self.ROTATE_TIME_IN_S)
			self.pub.publish(msg)
			sleep(self.ROTATE_TIME_IN_S+0.1)
		#angle_diff = start_angle - self.angle
		# sleep(2)
		# print("end angle", self.angle)

	def move_forward(self, speed_percentage=1, time_in_ms=1):
		msg = "tr,0,1,%s,%s" % (speed_percentage, time_in_ms)
		# print("actualy moves")
		self.pub.publish(msg)
		sleep(time_in_ms + time_in_ms*self.sleep_offset)


	def move_backward(self, speed_percentage=1, time_in_ms=1):
		msg = "tr,0,-1,%s,%s"  % (speed_percentage, time_in_ms)
		self.pub.publish(msg)
		sleep(time_in_ms + time_in_ms*self.sleep_offset)


	def move_left(self, speed_percentage=1, time_in_ms=1):
		msg = "tr,-1,0,%s,%s" % (speed_percentage, time_in_ms)
		self.pub.publish(msg)
		sleep(time_in_ms + time_in_ms*self.sleep_offset)


	def move_right(self, speed_percentage=1, time_in_ms=1):
		msg = "tr,1,0,%s,%s" % (speed_percentage, time_in_ms)
		self.pub.publish(msg)
		sleep(time_in_ms + time_in_ms*self.sleep_offset)

	def arc_left(self, speed_percentage=1, time_in_ms=1):
		msg = "arc,-1,-1,%s,%s" % (speed_percentage, time_in_ms)
		self.pub.publish(msg)
		sleep(time_in_ms + time_in_ms*self.sleep_offset)

	def arc_right(self, speed_percentage=1, time_in_ms=1):
		msg = "arc,1,1,%s,%s" % (speed_percentage, time_in_ms)
		self.pub.publish(msg)
		sleep(time_in_ms + time_in_ms*self.sleep_offset)


if __name__ == '__main__':
	rospy.init_node("test_movement")
	movement_calculator = MovementCalculator()
	while not rospy.is_shutdown():
		if movement_calculator.pub.get_num_connections() > 0:
			# movement_calculator.rotate_counterclockwise(25)
			movement_calculator.move_forward()
			#movement_calculator.rotate_clockwise(90)
			#move_left(pub)
			# pub.publish(things)
			# DO move stuff
			break
		movement_calculator.rate.sleep()
        
