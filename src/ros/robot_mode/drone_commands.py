#!/usr/bin/env python3

"""
.

must be python 3

ROS nodes need to be added to CMakeLists.txt
under catkin_install_python(PROGRAMS ...
"""

import rospy
from std_msgs.msg import String


ROSPY_RATE = 1


class TelloCommandDriver():
    def __init__(self):
        drone_commands_topic = '/drone_commands'
        self.my_tellos = list()
        self.my_tellos.append('0TQZK5DED02JWM')
        self.fly = FlyTello(my_tellos, get_status=True)
        self.sub = rospy.Subscriber(drone_commands_topic, String, self.drone_command_callback)


    def drone_command_callback(self, data):
        if data.data == "Land":
            pass
            # self.fly 
        elif data.data == "Idle":
            pass


if __name__ == '__main__':
    rospy.init_node("drone_commands")
    rate = rospy.Rate(ROSPY_RATE)
    tello_command_driver = TelloCommandDriver()
    print('drone commands subscriber listening!')
    rospy.spin()

