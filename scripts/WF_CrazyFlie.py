#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from hector_uav_msgs.srv import EnableMotors
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from wall_follower_multi_ranger import WallFollower
import time
import tf
import math
from _ast import IsNot
import logging
import sys
import numpy

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multi_ranger import MultiRanger
from cflib.utils.stabilization import Stabilization

from cflib.crazyflie.log import LogConfig


URI = 'radio://0/80/250K'


# Only output errors from the logging framework
#logging.basicConfig(level=logging.ERROR)

class WF_crazyflie:     
    # Callbacks
    front_range = 0.0
    right_range = 0.0
    altitude = 0.0
    state = "TAKE_OFF"
    current_heading = 0.0
    
    URI = 'radio://0/80/250K'
    
    if len(sys.argv) > 1:
        URI = sys.argv[1]
    
    # Only output errors from the logging framework
    logging.basicConfig(level=logging.ERROR)
        
            # Transition state and restart the timer
    def transition(self, newState):
        state = newState
        self.state_start_time = time.time()
        return state
        
    #def init(self):

    def data_received(self, timestamp, data, logconf):
        
        self.current_heading = math.radians(data['stabilizer.yaw'])
    
    def crazyFlieloop(self):
        wall_follower = WallFollower()
        wall_follower.init(0.3)
        twist = Twist()
        

        
        #log_config.data_received_cb.add_callback(self.data_received)
        
        cflib.crtp.init_drivers(enable_debug_driver=False)
        cf = Crazyflie(rw_cache='./cache')

        with SyncCrazyflie(self.URI, cf=cf) as scf:
            with MotionCommander(scf) as motion_commander:
                with MultiRanger(scf) as multi_ranger:
                    with Stabilization(scf) as stabilization:
                        keep_flying = True
                    
                        while keep_flying:
                            twist = wall_follower.wall_follower(multi_ranger.front,multi_ranger.right,stabilization.heading)
                            motion_commander._set_vel_setpoint(twist.linear.x,twist.linear.y,0,-1*math.degrees(twist.angular.z))
    
                            time.sleep(0.1)
                            
                            if multi_ranger.up < 0.2 and multi_ranger.up is not None:
                                keep_flying = False
                                
                        motion_commander.stop()
                        
                        print("demo terminated")
                            
                        
#         rate = rospy.Rate(10) # 10hz
#         while not rospy.is_shutdown():
#             if self.state == "TAKE_OFF":
#                 twist.linear.z = 0.1;
#                 if self.altitude > 0.5:
#                     self.state = self.transition("WALL_FOLLOWING")
#             if self.state =="WALL_FOLLOWING":                    
#                 twist = wall_follower.wall_follower(multi_ranger.front,multi_ranger.right, self.current_heading)
#             
#             pub.publish(twist)
#             rate.sleep()
#         
        

if __name__ == '__main__':
    

        
        
    WF_crazyflie = WF_crazyflie()
    
    WF_crazyflie.crazyFlieloop()
    #WF_crazyflie.init()
