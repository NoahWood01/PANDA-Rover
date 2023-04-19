from subprocess import call

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

    return ((right_angle + left_angle) / 2) - 90, 0
    


def align_orientation_to_box(controller, use_closest=True):
    """
    return True if aligned orientation, False if none found
    """
    theta, distance = get_orientation_to_box(controller, use_closest=use_closest)
    if not theta:
        return False
    rotate_angle = 180 - theta
    if rotate_angle > 0:
        controller.movement_calculator.rotate_clockwise(abs(rotate_angle))
    else:
        controller.movement_calculator.rotate_counterclockwise(abs(rotate_angle))
    return True

    
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
    return theta
  
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

    if (
        
        left_index > (len(controller.front_lidar_scan) / 2) and
        right_index < (len(controller.front_lidar_scan) / 2) and
        controller.front_lidar_scan[(len(controller.front_lidar_scan) / 2) + index_cone_window] > 500 and
        controller.front_lidar_scan[(len(controller.front_lidar_scan) / 2) - index_cone_window] > 500
    ):
        print("found")
        return True 
    return False

def check_final_alignment(controller):
    index_cone_window = 20
    right_left_error_window = 25

    if controller.front_lidar_scan[(len(controller.front_lidar_scan) / 2)] < 900:
        return False

    # Checking left edge
    left_index = float(0)
    right_index = float(0)
    right_distance = float(0)
    left_distance = float(0)
    scans = list(controller.front_lidar_scan)

    for index, distance in enumerate(scans):
        if distance < 500:
            right_index = index
            right_distance = distance
            break

    for index in range(len(scans) -1, 0, -1):
        if scans[index] < 500:
            left_index = index
            left_distance = scans[index]
            break

    if (
        
        # left_index > (len(controller.front_lidar_scan) / 2) and
        # right_index < (len(controller.front_lidar_scan) / 2) and
        abs(left_distance - right_distance) < right_left_error_window and
        controller.front_lidar_scan[(len(controller.front_lidar_scan) / 2) + index_cone_window] > 500 and
        controller.front_lidar_scan[(len(controller.front_lidar_scan) / 2) - index_cone_window] > 500
    ):
        print("Aligned and rdy to move")
        return True 
    return False

def get_left_right_distance_offset(controller):
    # Checking left edge
    left_index = float(0)
    right_index = float(0)
    right_distance = float(0)
    left_distance = float(0)
    scans = list(controller.front_lidar_scan)

    for index, distance in enumerate(scans):
        if distance < 500:
            right_distance = distance
            break

    for index in range(len(scans) -1, 0, -1):
        if scans[index] < 500:
            left_distance = scans[index]
            break
    return left_distance - right_distance


def check_if_outside_box(controller):
    '''
    returns if rover is inside the box
    '''
    right_distance = controller.front_lidar_scan[0]
    left_distance = controller.front_lidar_scan[len(controller.front_lidar_scan) - 1]

    if right_distance > 800 and left_distance > 800:
        return True
    return False

def check_orientation_inside_box(controller):
    right_distance = controller.front_lidar_scan[0]
    left_distance = controller.front_lidar_scan[len(controller.front_lidar_scan) - 1]

    left_right_difference = left_distance - right_distance

    right_edge_index = float(0)
    left_edge_index = float(0)
    for index in range((len(controller.front_lidar_scan) - 1) / 2, 0, -1):
        if controller.front_lidar_scan[index] < 1000:
            right_edge_index = index
            break
    
    for index in range((len(controller.front_lidar_scan) - 1) / 2, (len(controller.front_lidar_scan) - 1), 1):
        if controller.front_lidar_scan[index] < 1000:
            left_edge_index = index
            break

    change_in_right_from_index_middle = abs((len(controller.front_lidar_scan) / 2) - right_edge_index)
    change_in_left_from_index_middle = abs(left_edge_index - (len(controller.front_lidar_scan) / 2))

    left_right_index_difference = float(0)
    left_right_index_difference = change_in_left_from_index_middle - change_in_right_from_index_middle
    left_right_angle_difference = ((left_right_index_difference / 2) * 180) / len(controller.front_lidar_scan)
    return left_right_angle_difference, left_right_difference


def make_opening_aligned(controller):
    while not check_if_aligned_with_opening_in_cone(controller):
        controller.movement_calculator.move_left(speed_percentage=0.1,time_in_ms=ITERATION_TIME)
        box_angle_offset = get_angle_offset_of_closest_box(controller)
        if box_angle_offset > 5:
            controller.movement_calculator.rotate_counterclockwise(abs(box_angle_offset))
        if box_angle_offset < 5:
            controller.movement_calculator.rotate_clockwise(abs(box_angle_offset))
        reached_distance, min_scan =_scan_iteration(controller)
        if min_scan < 200:
            controller.movement_calculator.move_backward(speed_percentage=0.2,time_in_ms=ITERATION_TIME)
        if min_scan > 300:
            controller.movement_calculator.move_forward(speed_percentage=0.2,time_in_ms=ITERATION_TIME)
        
    prev_left_right_offset = float(1)
    while not check_final_alignment(controller):
        if prev_left_right_offset > 0:
            controller.movement_calculator.move_left(speed_percentage=0.1,time_in_ms=ITERATION_TIME)
        else:
            controller.movement_calculator.move_right(speed_percentage=0.1,time_in_ms=ITERATION_TIME)
        prev_left_right_offset = get_left_right_distance_offset(controller)
        box_angle_offset = get_angle_offset_of_closest_box(controller)
        if box_angle_offset > 2:
            controller.movement_calculator.rotate_counterclockwise(abs(box_angle_offset))
        if box_angle_offset < 2:
            controller.movement_calculator.rotate_clockwise(abs(box_angle_offset))
        reached_distance, min_scan =_scan_iteration(controller)
        if min_scan < 200:
            controller.movement_calculator.move_backward(speed_percentage=0.2,time_in_ms=ITERATION_TIME)
        if min_scan > 300:
            controller.movement_calculator.move_forward(speed_percentage=0.2,time_in_ms=ITERATION_TIME)

def move_through_box(controller):
    while check_if_outside_box(controller):
        controller.movement_calculator.move_forward(speed_percentage=0.2,time_in_ms=0.1)

    while check_if_outside_box(controller):
        controller.movement_calculator.move_forward(speed_percentage=0.2,time_in_ms=0.1)

    while not check_if_outside_box(controller):
        controller.movement_calculator.move_forward(speed_percentage=0.3,time_in_ms=ITERATION_TIME)
        inside_box_angle, inside_box_left_right_distance = check_orientation_inside_box(controller)
        if inside_box_angle > 0:
            controller.movement_calculator.rotate_counterclockwise(abs(inside_box_angle) / 2)
        if inside_box_angle < 0:
            controller.movement_calculator.rotate_clockwise(abs(inside_box_angle) / 2)
        if inside_box_left_right_distance < -50 and inside_box_left_right_distance > -1000:
            controller.movement_calculator.move_right(speed_percentage=0.2,time_in_ms=ITERATION_TIME)
        if inside_box_left_right_distance > 50 and inside_box_left_right_distance < 1000:
            controller.movement_calculator.move_left(speed_percentage=0.2,time_in_ms=ITERATION_TIME)

    controller.movement_calculator.move_forward(speed_percentage=0.5,time_in_ms=3)

