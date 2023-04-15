#!/usr/bin/env python3

"""
.

must be python 3

ROS nodes need to be added to CMakeLists.txt
under catkin_install_python(PROGRAMS ...
"""

import rospy
from std_msgs.msg import String
from fly_tello import FlyTello
from time import sleep
import threading

ROSPY_RATE = 1

class TelloCommandDriver():
    def __init__(self):
        drone_commands_topic = '/drone_commands'
        self.my_tellos = list()
        self.my_tellos.append('0TQZK5DED02JWM')
        self.fly = FlyTello(self.my_tellos, get_status=True)
        self.sub = rospy.Subscriber(drone_commands_topic, String, self.drone_command_callback)
        print("callback check")
        self.t1 = threading.Thread(target=self.f)
        self.hover = False
    
    # def f(self):
    #     while self.hover:
    #         self.fly.down(dist=20)
    #         sleep(2)


    def drone_command_callback(self, data):
        print(data)

        if data.data == "Land":
            print("land")
            pass
            # self.fly 
        elif data.data == "Hover":
            print("Starting hover thread")
            for i in range(20):
                self.fly.down(dist=20)
                sleep(1)
            # self.hover = True
            # while self.hover:
            #     self.fly.flip("forward")
            #     sleep(4)
            # if not self.hover and not self.t1.is_alive():
            #     self.hover = True
            #     self.t1.start()
        elif data.data == "End Hover":
            print("Ending hover thread")
            self.hover = False
            self.t1.join()
            print("Hover thread done")
        elif data.data == "Takeoff":
            first_yaw = self.fly.get_status("yaw", tello=1, sync=True) # only record this the very first takeoff
            print("first yaw: " + first_yaw)
            self.fly.takeoff()
            self.fly.up(dist=20)
            self.fly.forward(dist=30)   
        elif data.data == "Backward":
            self.fly.back(dist=80)    
            sleep(5)
                
            self.fly.straight_from_pad(x=0, y=0, z=50, speed=100, pad='m4')
            sleep(4)
            self.fly.straight_from_pad(x=0, y=0, z=50, speed=100, pad='m4')
            sleep(4)
            self.fly.straight_from_pad(x=0, y=0, z=25, speed=100, pad='m4') #x = 5 for going back, x = 0 for going forward
            sleep(4)

            self.fly.straight_from_pad(x=-5, y=0, z=25, speed=100, pad='m4')
            sleep(4)
            self.fly.straight_from_pad(x=-3, y=3, z=25, speed=100, pad='m4')
            sleep(2)
            
            # land_yaw = self.fly.get_status("yaw", tello=1, sync=True)
            # print("land yaw: " + land_yaw)
            # diff_yaw = int(land_yaw) - int(tello_takeoff.first_yaw)
            # if diff_yaw > 0:
            #     fly.rotate_ccw(angle=diff_yaw, tello=1)
            #     time.sleep(2)
            # if diff_yaw < 0: 
            #     fly.rotate_cw(angle=abs(diff_yaw), tello=1) 
            #     time.sleep(2)   

            self.fly.land() 
        elif data.data == "Forward":
            self.fly.up(dist=30) 
            self.fly.forward(dist=30)   

            sleep(3)
            self.fly.straight_from_pad(x=0, y=0, z=50, speed=100, pad='m4')
            sleep(4)
            self.fly.straight_from_pad(x=0, y=0, z=50, speed=100, pad='m4')
            sleep(4)
            self.fly.straight_from_pad(x=0, y=0, z=25, speed=100, pad='m4') #x = 5 for going back, x = 0 for going forward
            sleep(4)

            self.fly.straight_from_pad(x=-8, y=0, z=25, speed=100, pad='m4')
            sleep(4)       
            self.fly.straight_from_pad(x=-7, y=3, z=25, speed=100, pad='m4')
            sleep(2)
            
            # land_yaw = self.fly.get_status("yaw", tello=1, sync=True)
            # print("land yaw: " + land_yaw)
            # diff_yaw = int(land_yaw) - int(tello_takeoff.first_yaw)
            # if diff_yaw > 0:
            #     fly.rotate_ccw(angle=diff_yaw, tello=1)
            #     time.sleep(2)
            # if diff_yaw < 0: 
            #     fly.rotate_cw(angle=abs(diff_yaw), tello=1) 
            #     time.sleep(2)   
            #fly.set_rc(left_right=0, forward_back=0, up_down=0, yaw=int(fixed_yaw))
            #time.sleep(1)
            
            #fly.straight_from_pad(x=0, y=0, z=25, speed=100, pad='m4')
        #    #fly.flip(direction='back')
        ##    fly.reorient(height=25, pad='m4')
            self.fly.land()

if __name__ == '__main__':
    rospy.init_node("drone_commands")
    rate = rospy.Rate(ROSPY_RATE)
    tello_command_driver = TelloCommandDriver()
    print('drone commands subscriber listening!')
    rospy.spin()

