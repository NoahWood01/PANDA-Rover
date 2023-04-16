# from util import align_orientation_to_box, make_opening_aligned
ANGLE_OFFSET_WINDOW = 20 # in Index not DEGREES 1 index = 0.3 degreesish
ITERATION_TIME = 0.1
APPROACH_BOX_DISTANCE = 250
DISTANCE_BOX_DETECTION_WINDOW = 20

def get_orientation_to_box(controller, use_closest=True):
    """
    return a box theta, box distance
    """
    if len(controller.lidar_boxes) == 0 or not controller.lidar_boxes:
        return
    print("Finding closest")
    if not use_closest:
        return controller.lidar_boxes.theta, controller.lidar_boxes.distance

    min_distance_box = controller.lidar_boxes[0]
    for box in controller.lidar_boxes:
        if box.distance < min_distance_box.distance:
            min_distance_box = box

    return min_distance_box.theta, min_distance_box.distance


def get_orientation_to_box_filtered(controller):
    threshold = 500
    
    # filtered_scans = filter(lambda x: x < threshold, controller.front_lidar_scan)
    # Checking left edge
    left_angle = float(0)
    right_angle = float(0)
    scans = list(controller.front_lidar_scan)

    for index, distances in enumerate(scans):
        if distances < threshold:
            right_angle = (180 * index) / len(controller.front_lidar_scan)
            break

    for index in range(len(scans) -1, 0, -1):
        if scans[index] < threshold:
            left_angle = (180 * index) / len(controller.front_lidar_scan)
            break

    
    print("left_angle, right_angle", str(left_angle), str(right_angle), str((right_angle + left_angle) / 2))
    # print((right_angle + left_angle) / 2)
    return ((right_angle + left_angle) / 2) - 90, 0
    


def align_orientation_to_box(controller, use_closest=True):
    theta, distance = get_orientation_to_box(controller, use_closest=use_closest)
    rotate_angle = 180 - theta
    if rotate_angle > 0:
        controller.movement_calculator.rotate_clockwise(abs(rotate_angle))
    else:
        controller.movement_calculator.rotate_counterclockwise(abs(rotate_angle))

    
# def get_minimum_lidar_distance(controller):
#     for distances in for index, distances in enumerate(controller.front_lidar_scan):


def _scan_iteration(controller):
	reached_distance = False
	min_scan = float('inf')
	for scan_point in controller.front_lidar_scan:
		if scan_point < APPROACH_BOX_DISTANCE:
			
			reached_distance = True
		if scan_point < min_scan:
			min_scan = scan_point
	if reached_distance:
		print("reached box threshold. min at: %s" % min_scan)
	return reached_distance, min_scan


def check_is_opening_found(controller):
    if controller.front_lidar_scan[ len(controller.front_lidar_scan) / 2] < 1500:
        return False
    return True

def get_angle_offset_of_closest_box(controller):
    theta, distance = get_orientation_to_box_filtered(controller)
    print(theta)
    return theta

def check_if_aligned_with_opening(controller):
    # Checking left edge
    left = float(0)
    right = float(0)
    for index, distances in enumerate(controller.front_lidar_scan):
        if distances < 500:
            left = distances
            break

    for index, distances in enumerate(controller.front_lidar_scan[::-1]):
        if distances < 500:
            right = distances
            break

    left_offset = (len(controller.front_lidar_scan) / 2) - left
    right_offset = right - (len(controller.front_lidar_scan) / 2)

    print('left, right')
    print(left, right, abs(left - right))
    # print(right_offset)
    # if left_offset <= abs(right_offset) + ANGLE_OFFSET_WINDOW and left_offset >= abs(right_offset) - ANGLE_OFFSET_WINDOW and controller.front_lidar_scan[ len(controller.front_lidar_scan) / 2] > 1500:
    #     return True
    if abs(left - right) < DISTANCE_BOX_DETECTION_WINDOW and controller.front_lidar_scan[ len(controller.front_lidar_scan) / 2] > 1500:
        return True
    return False
    
def check_if_aligned_with_opening_in_cone(controller):
    index_cone_window = 20

    if controller.front_lidar_scan[(len(controller.front_lidar_scan) / 2)] < 900:
        return False

    # Checking left edge
    left_index = float(0)
    right_index = float(0)
    scans = list(controller.front_lidar_scan)

    for index, distances in enumerate(scans):
        if distances < 500:
            right_index = index
            break

    for index in range(len(scans) -1, 0, -1):
        if scans[index] < 500:
            left_index = index
            break

    print("left_index: ", str(left_index), "right_index: ", str(right_index))
    print("mid distance", str(controller.front_lidar_scan[(len(controller.front_lidar_scan) / 2)]), "left vision distance: ", str(controller.front_lidar_scan[(len(controller.front_lidar_scan) / 2) + index_cone_window]), "right vision distance: ", str(controller.front_lidar_scan[(len(controller.front_lidar_scan) / 2) - index_cone_window]))
    if (
        
        left_index > (len(controller.front_lidar_scan) / 2) and
        right_index < (len(controller.front_lidar_scan) / 2) and
        controller.front_lidar_scan[(len(controller.front_lidar_scan) / 2) + index_cone_window] > 500 and
        controller.front_lidar_scan[(len(controller.front_lidar_scan) / 2) - index_cone_window] > 500
    ):
        print("found")
        return True 
    return False

def make_opening_aligned(controller):
    while not check_if_aligned_with_opening_in_cone(controller):
        controller.movement_calculator.move_left(speed_percentage=0.1,time_in_ms=ITERATION_TIME)
        box_angle_offset = get_angle_offset_of_closest_box(controller)
        print(box_angle_offset)
        if box_angle_offset > 5:
            controller.movement_calculator.rotate_counterclockwise(abs(box_angle_offset))
        if box_angle_offset < 5:
            controller.movement_calculator.rotate_clockwise(abs(box_angle_offset))
        reached_distance, min_scan =_scan_iteration(controller)
        if min_scan < 200:
            controller.movement_calculator.move_backward(speed_percentage=0.2,time_in_ms=ITERATION_TIME)
        if min_scan > 300:
            controller.movement_calculator.move_forward(speed_percentage=0.2,time_in_ms=ITERATION_TIME)
        
    """
    print('Detected some opening')
    while not check_if_aligned_with_opening(controller):
        controller.movement_calculator.move_left(speed_percentage=0.1,time_in_ms=ITERATION_TIME)
        box_angle_offset = get_angle_offset_of_closest_box(controller)
        if box_angle_offset > 5:
            controller.movement_calculator.rotate_clockwise(abs(box_angle_offset))
        if box_angle_offset < 5:
            controller.movement_calculator.rotate_counterclockwise(abs(box_angle_offset))
        if min_scan < 200:
            controller.movement_calculator.move_backward(speed_percentage=0.2,time_in_ms=ITERATION_TIME)
        if min_scan > 300:
            controller.movement_calculator.move_forward(speed_percentage=0.2,time_in_ms=ITERATION_TIME)
    
    ANGLE_OFFSET_WINDOW = 10
    while not check_if_aligned_with_opening(controller):
        controller.movement_calculator.move_left(speed_percentage=0.1,time_in_ms=ITERATION_TIME)
        box_angle_offset = get_angle_offset_of_closest_box(controller)
        if box_angle_offset > 3:
            controller.movement_calculator.rotate_clockwise(abs(box_angle_offset))
        if box_angle_offset < 3:
            controller.movement_calculator.rotate_counterclockwise(abs(box_angle_offset))
        if min_scan < 200:
            controller.movement_calculator.move_backward(speed_percentage=0.2,time_in_ms=ITERATION_TIME)
        if min_scan > 300:
            controller.movement_calculator.move_forward(speed_percentage=0.2,time_in_ms=ITERATION_TIME)
    """
    
    print('Found')
        # move Left

