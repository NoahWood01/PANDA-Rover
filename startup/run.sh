#!/bin/bash

# This will be the single start script to begin everything.
# Assumption here is that everything must autonomously start on power up
# We will set this to run on boot


# TODO on start
# Boot all cameras/accessories
# Connect Rover and Drone
# run code


function startUpLaunches () {
    source /home/panda/.bashrc
    source /opt/ros/melodic/setup.bash
    source /home/panda/catkin_ws/devel/setup.bash

    # Start stuff goes here
    echo "Startup initiated"

    roscore &>/dev/null &
    # wait for ros boot up
    echo "roscore up"
    sleep 5
    roslaunch realsense2_camera rs_camera.launch &>/dev/null &
    sleep 3
    echo "realsense ros node launched"

    sleep 1
    echo "launcing panda nodes"
    roslaunch panda_rover panda.launch &>/dev/null &
    sleep 5
    echo "Init Lidar"
    roslaunch rplidar_ros rplidar.launch &>/dev/null &
    sleep 3

    roslaunch box_detection rplidar_boxes.launch &>/dev/null &
    echo "Init Motors"
    roslaunch dynamixel_sdk_examples motors.launch &>/dev/null &
    sleep 3
    echo "Launching Done"
	
    echo "Init upward camera"
    roslaunch jetbot_ros upwardcam.launch &>/dev/null &
    sleep 3
    echo "Upward camera launched"

    echo "Init QR code reader script"
    roslaunch zbar_ros qr_reader.launch &>/dev/null &
    sleep 3
    echo "QR code reader script launched"
    # This is pubbed in the Controller.py script

    echo "Init IMU script"
    roslaunch imu_bno055 imu.launch &>/dev/null &
    sleep 3
    echo "IMU has launched"
    # data is output in topic "/imu/data", I will add this to controller  - jules
    
}

function startUpDocker () {
    echo "Launching Docker Container and Tello driver"
    # dont make this a background process
    # can alternately use:
    # ./docker_ws/test_container.sh
    # docker exec tello_container bash -c "source /opt/ros/melodic/setup.bash && source /root/catkin_ws/devel/setup.bash && roslaunch tello_driver tello_node.launch"
    exec sh ~/docker_ws/test_container.sh
}


if [[ $# -eq 0 ]] || [[ $1 == "help" ]] ; then
    echo "Run Tests ./run.sh test"
    echo "Run Startup ./run.sh start"
    echo "running startup"

    startUpLaunches
    startUpDocker

    exit 0
fi

name=$1
if [[ $name == "test" ]]; then
    echo "Running test.py"
    source .venv/bin/activate
    # exec python3 src/test.py
    exec roslaunch rplidar_ros rplidar.launch
    exec rosrun dynamixel_sdk_examples read_write_node.py
elif [[ $name == "start" ]]; then
    startUpLaunches
    startUpDocker
elif [[ $name == "noDocker" ]]; then
    startUpLaunches
fi
