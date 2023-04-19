#!/usr/bin/env bash
  
xhost +local:root

docker run \
    --interactive \
    --tty \
    --rm \
    --privileged \
    --network host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --name="tello_container" \
    tello:latest \
    bash -c "source /opt/ros/melodic/setup.bash && source /root/catkin_ws/devel/setup.bash && roslaunch tello_driver tello_node.launch"

xhost -local:root

