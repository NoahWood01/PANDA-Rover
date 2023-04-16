from . import Tello
tello = Tello()
# Connect to the drone
tello.connect()

# Enable mission pads
tello.enable_mission_pads()

# Set the direction of the mission pad detection (in this example, downward)
tello.set_mission_pad_detection_direction(0)

# Takeoff
tello.takeoff()

# Move forward until a mission pad is detected
while tello.get_mission_pad_id() == -1:
    tello.move_forward(30)
    
# Stop the drone
tello.send_control_command("stop") # or tello.send_control_command("rc 0 0 0 0")

# Get the ID of the detected mission pad
mission_pad_id = tello.get_mission_pad_id()

# Get the distance from the mission pad in the X, Y, and Z directions
distance_x = tello.get_mission_pad_distance_x()
distance_y = tello.get_mission_pad_distance_y()
distance_z = tello.get_mission_pad_distance_z()

# Try:
# Adjust the distances to move the drone closer to the center of the mission pad
#mission_pad_size = 290 # ? mission pad is 290mm x 290mm
#distance_x -= mission_pad_size / 2
#distance_y -= mission_pad_size / 2

# Move to the mission pad
tello.move_left(distance_x)
tello.move_backward(distance_y)
tello.move_down(distance_z)

# OR fly to the "x", "y", and "z" coordinates of the mission pad detected
# Example: SDK instruction: go 0 0 100 60 m1.. 
# The Tello EDU has detected the mission pad marked 1. The drone flies at a speed of 60 cm/s 
# to the coordinate point (0, 0, 100) in the mission pad coordinate system. 
#tello.go_xyz_speed_mid(0, 0, 100, 60, mission_pad_id)

# OR try 
#tello.go_xyz_speed_mid(distance_x, distance_y, distance_z, 60, mission_pad_id)

# Land on the mission pad
tello.land()

# Disconnect from the drone
tello.end()


