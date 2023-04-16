from util import align_orientation_to_box
from random import randint

def search(controller):
    # TODO maybe alter logic to account for seen boxes?
    # right now just goes to closest seen box
    is_box_found = False
    is_box_found = align_orientation_to_box(controller, use_closest=True)
    while not is_box_found:
        # TODO optimize this if we have time using imu and location mapping

        
        # TODO add out of bounds checking if we have time
        # make random movements until a box is found
        random_movement(controller)
        is_box_found = align_orientation_to_box(controller, use_closest=True)


def random_movement(controller):
    random_choice = randint(0,2)
    # want to only move forward so we dont hit a box
    # match random_choice:
    #     case 0:
    #         controller.movement_calculator.rotate_clockwise(get_random_theta_rotation())
    #     case 1:
    #         controller.movement_calculator.rotate_counterclockwise(get_random_theta_rotation())
    #     case 2:
    #         controller.movement_calculator.move_forward()


def get_random_theta_rotation():
    return randint(0, 90)

