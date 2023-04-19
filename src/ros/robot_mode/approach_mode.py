"""
Helper functions for Robot modes
"""

from subprocess import call
import threading
from util import align_orientation_to_box, make_opening_aligned, ITERATION_TIME, _scan_iteration, APPROACH_BOX_DISTANCE, get_orientation_to_box_filtered, move_through_box
from time import sleep



def approach(controller):
	"""
	assumption: already facing the box.
	"""
	print("Locating")
	while len(controller.lidar_boxes) == 0: 
		print("no boxes detected")
		sleep(1)

	# TODO SEARCH DOES THIS DELETE WHEN DONE TESTING
	align_orientation_to_box(controller)


	print("Approaching")
	_move_to_box(controller)
	print("Align with opening")
	make_opening_aligned(controller)
	print("Aligned, sending drone")
	sleep(5)
	controller.drone_command_pub.publish("Takeoff")
	sleep(15)
	
	# If Correct box
	if self.drone_qr_code == self.next_box_id:
		controller.drone_command_pub.publish("Hover")
		print("Move through box")
		move_through_box(controller)
		controller.drone_command_pub.publish("End Hover")
		print("Correct box found, drone forward")
		controller.drone_command_pub.publish("Forward")
	else:
		print("Incorrect box found, drone backward")
		controller.drone_command_pub.publish("Backward")
	
	# This is the case where incorrect box is found, so drone moves backward and lands

#
	# sleep(5) # Here we would want to set hover = False when decision is made based on barcode_drone

	# Here we decide whether to recall drone or move through based on barcode_drone topic

	# controller.drone_command_pub.publish("End Hover")
# sleep(1)

	# This is the case where correct box is found, so drone moves forward and lands
	# print("Correct box found, drone forward")
	# controller.drone_command_pub.publish("Forward")






def _move_to_box(controller):
	"""
	move to the x distance to the box. Based on minimum from lidar
	"""
	reached_distance, min_scan = _scan_iteration(controller)
	while not reached_distance:
		controller.movement_calculator.move_forward(speed_percentage=0.1,time_in_ms=ITERATION_TIME)
		reached_distance, min_scan = _scan_iteration(controller)
		print(min_scan)



