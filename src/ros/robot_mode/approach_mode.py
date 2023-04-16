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
	# print("Locating")
	# while len(controller.lidar_boxes) == 0: 
	# 	sleep(1)

	# align_orientation_to_box(controller)
	# print("Approaching")
	# _move_to_box(controller)
	# print("Align with opening")
	# make_opening_aligned(controller)
	print("Aligned, sending drone")
	exit_code = call("python3 ./tello_takeoff.py", shell=True)
	controller.drone_command_pub.publish("Land")

	# hover = False
	# def f(): 
	# 	while hover:
	# 		exit_code = call("python3 ./tello_hover.py", shell=True)
	# 		sleep(2)
	# t1 = threading.Thread(target=f)

	# print("Starting hover thread")
	# hover = True
	# t1.start()

	# sleep(5) # Here we would want to set hover = False when decision is made based on barcode_drone
	# hover = False
	# t1.join()
	# print("Hover thread done")

	# This is the case where correct box is found, so drone moves forward and lands
	# print("Correct box found, drone forward")
	# exit_code = call("python3 ./tello_forward_land.py", shell=True)

	# This is the case where incorrect box is found, so drone moves backward and lands
	print("Incorrect box found, drone backward")
	exit_code = call("python3 ./tello_backward_land.py", shell=True)

	# Here we decide whether to recall drone or move through based on barcode_drone topic
	#print("Move through box")
	#move_through_box(controller)


def _move_to_box(controller):
	"""
	move to the x distance to the box. Based on minimum from lidar
	"""
	reached_distance, min_scan = _scan_iteration(controller)
	while not reached_distance:
		controller.movement_calculator.move_forward(speed_percentage=0.1,time_in_ms=ITERATION_TIME)
		reached_distance, min_scan = _scan_iteration(controller)
		print(min_scan)



