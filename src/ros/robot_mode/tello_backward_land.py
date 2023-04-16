import time
import tello_takeoff
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
    fly.back(dist=40)    
    time.sleep(5)
        
    fly.straight_from_pad(x=0, y=0, z=50, speed=100, pad='m4')
    time.sleep(4)
    fly.straight_from_pad(x=0, y=0, z=50, speed=100, pad='m4')
    time.sleep(4)
    fly.straight_from_pad(x=0, y=0, z=25, speed=100, pad='m4') #x = 5 for going back, x = 0 for going forward
    time.sleep(4)

    fly.straight_from_pad(x=-5, y=0, z=25, speed=100, pad='m4')
    time.sleep(4)
    fly.straight_from_pad(x=-3, y=3, z=25, speed=100, pad='m4')
    time.sleep(2)
    
    land_yaw = fly.get_status("yaw", tello=1, sync=True)
    print("land yaw: " + land_yaw)
    diff_yaw = int(land_yaw) - int(tello_takeoff.first_yaw)
    if diff_yaw > 0:
        fly.rotate_ccw(angle=diff_yaw, tello=1)
        time.sleep(2)
    if diff_yaw < 0: 
        fly.rotate_cw(angle=abs(diff_yaw), tello=1) 
        time.sleep(2)   
    #fly.set_rc(left_right=0, forward_back=0, up_down=0, yaw=int(fixed_yaw))
    #time.sleep(1)
    
    #fly.straight_from_pad(x=0, y=0, z=25, speed=100, pad='m4')
#    #fly.flip(direction='back')
##    fly.reorient(height=25, pad='m4')
    fly.land()
    fly.tello_mgr.wait_sync()
    fly.tello_mgr.close_connections()
#    time.sleep(1)
#    
#    print(int(fly.get_status(key="h", tello=1, sync=True)))
#    
#    # If it fails to land on mission pad:
#    if int(fly.get_status(key="h", tello=1, sync=True)) < 0:
#        fly.takeoff()
#        fly.up(dist=90)
#        fly.straight_from_pad(x=2, y=0, z=50, speed=100, pad='m4')
#        fly.straight_from_pad(x=0, y=0, z=25, speed=100, pad='m4') #x = 5 for going back, x = 0 for going forward
#            
#        #fly.set_rc(left_right=0, forward_back=0, up_down=0, yaw=fixed_yaw)
#        fly.land()
#        time.sleep(1)
#        
#        print(int(fly.get_status(key="h", tello=1, sync=True)))
#        
#        # If it fails to land on mission pad again:
#        while int(fly.get_status(key="h", tello=1, sync=True)) <= 0:
#            fly.takeoff()
#            fly.up(dist=90)
#            fly.straight_from_pad(x=0, y=0, z=50, speed=100, pad='m4')
#            fly.straight_from_pad(x=0, y=0, z=25, speed=100, pad='m4') #x = 5 for going back, x = 0 for going forward
#            
#            #fly.set_rc(left_right=0, forward_back=0, up_down=0, yaw=fixed_yaw)
#            fly.land()
#            time.sleep(1)
#            
#            print(int(fly.get_status(key="h", tello=1, sync=True)))