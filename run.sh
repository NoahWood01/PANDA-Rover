#!/bin/bash

set -e -u -o pipefail

# This will be the single start script to begin everything.
# Assumption here is that everything must autonomously start on power up
# We will set this to run on boot


# TODO on start
# Boot all cameras/accessories
# Connect Rover and Drone
# run code


if [[ $# -eq 0 ]] || [[ $1 == "help" ]] ; then
    echo "Run Tests ./run.sh test"
    echo "Run Startup ./run.sh start"
    exit 0
fi

name=$1
if [[ $name == "test" ]]; then
    echo "Running test.py"
    source .venv/bin/activate
    exec python3 src/test.py
elif [[ $name == "start" ]]; then
    # Start stuff goes here
    echo "Startup initiated"

    roscore &>/dev/null &
    # wait for ros boot up
    echo "roscore up"
    sleep 5
    roslaunch realsense2_camera rs_camera.launch &>/dev/null &
    echo "realsense ros node launched"
    sleep 1

    echo "Do some stuff like:"
    echo "source .venv/bin/activate"
    echo "exec python3 src/main.py"
fi