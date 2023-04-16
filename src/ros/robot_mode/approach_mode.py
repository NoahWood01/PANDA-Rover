"""
Helper functions for Robot modes
"""


from util import align_orientation_to_box, make_opening_aligned, ITERATION_TIME, _scan_iteration, APPROACH_BOX_DISTANCE, get_orientation_to_box_filtered
from time import sleep



def approach(controller):
	"""
	assumption: already facing the box.
	"""
	print("Locating")
	while len(controller.lidar_boxes) == 0: 
		sleep(`1)`
	# while True:
	# 	print(str(controller.front_lidar_scan[0]), str(controller.front_lidar_scan[-1]))

	# align_orientation_to_box(controller)
	print("approaching")
	# _move_to_box(controller)
	print("Align with opening")
	make_opening_aligned(controller)



def _move_to_box(controller):
	"""
	move to the x distance to the box. Based on minimum from lidar
	"""
	reached_distance, min_scan = _scan_iteration(controller)
	while not reached_distance:
		controller.movement_calculator.move_forward(speed_percentage=0.1,time_in_ms=ITERATION_TIME)
		reached_distance, min_scan = _scan_iteration(controller)
		print(min_scan)



