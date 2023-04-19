import time
from fly_tello import FlyTello
my_tellos = list()


#
# SIMPLE EXAMPLE - SINGLE TELLO WITH MISSION PAD
#
# SETUP: Tello on Mission Pad, facing in direction of the pad - goes max 50cm left and 100cm forward - takes ~45sec
#

#
# MAIN FLIGHT CONTROL LOGIC
#

# Define the Tello's we're using, in the order we want them numbered
my_tellos.append('0TQZK5DED02JWM')  # 1-Yellow
# my_tellos.append('0TQDFC6EDB4398')  # 2-Blue
# my_tellos.append('0TQDFC6EDBH8M8')  # 3-Green
# my_tellos.append('0TQDFC7EDB4874')  # 4-Red

# Control the flight
with FlyTello(my_tellos, get_status=True) as fly:
    fly.rotate_cw(angle=0, tello=1)
