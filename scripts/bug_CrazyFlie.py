#!/usr/bin/env python
# license removed for brevity
'''
import rospy
from std_msgs.msg import String
'''


from geometry_msgs.msg import Twist
'''

from gazebo_msgs.msg import ModelStates
from hector_uav_msgs.srv import EnableMotors
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
'''


import time
#import tf
import math
from _ast import IsNot
import logging
import sys
import numpy as np

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multi_ranger import MultiRanger
from cflib.utils.stabilization import Stabilization
from cflib.crazyflie.syncLogger import SyncLogger

from cflib.crazyflie.log import LogConfig

from wall_following_controller import WallFollowerController
from com_controller import ComController
from com_angle_controller import ComAngleController
from com_angle_loop_controller import ComAngleLoopController




## OPTITRACK stuff
#Callbacks for optitrack
from NatNetClient import NatNetClient
pos = []
check = 0

def receiveNewFrame( frameNumber, markerSetCount, unlabeledMarkersCount, rigidBodyCount, skeletonCount,
                    labeledMarkerCount, latency, timecode, timecodeSub, timestamp, isRecording, trackedModelsChanged ):
    check = 1

def receiveRigidBodyFrame( id, position, rotation ):
    global pos
    if id == 1:
        pos = position
        #print( "Received frame for rigid body", id )

# Intialization of Optirack
streamingClient = NatNetClient()
streamingClient.newFrameListener = receiveNewFrame
streamingClient.rigidBodyListener = receiveRigidBodyFrame
streamingClient.run()

DISTANCE_TO_TRAVEL = 8.0
def logicIsCloseTo( real_value = 0.0, checked_value =0.0, margin=0.05):

    if real_value> checked_value-margin and real_value< checked_value+margin:
        return True
    else:
        return False
def wraptopi(number):
    return  ( number + np.pi) % (2 * np.pi ) - np.pi

# Only output errors from the logging framework
#logging.basicConfig(level=logging.ERROR)

class WF_crazyflie:
    # Callbacks
    front_range = 0.0
    right_range = 0.0
    altitude = 0.0
    state = "TAKE_OFF"
    current_heading = 0.0
    state_start_time = 0
    #Crazyflie 1
    #URI = 'radio://0/40/250K/E7E7E7E7EA'
    #Crazyflie 2
    URI = 'radio://0/60/250K/E7E7E7E7EB'
    #Crazyflie 3
    #URI = 'radio://0/80/250K/E7E7E7E7E7'
    #Crazyflie 4
    #URI = 'radio://0/50/250K/E7E7E7E7EC'

    angle_to_goal = 0.0
    distance_to_goal = 0.0
    
    goal_coord_x = [4, 0]
    goal_coord_y = [-2, 0]
    
    coord_index = 0

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
        #wall_follower = WallFollowerController()
        #wall_follower.init(0.5)
        twist = Twist()

        #log_config.data_received_cb.add_callback(self.data_received)

        cflib.crtp.init_drivers(enable_debug_driver=False)
        cf = Crazyflie(rw_cache='./cache')

        lg_states = LogConfig(name='kalman_states', period_in_ms=100)
        lg_states.add_variable('stabilizer.yaw')
        lg_states.add_variable('kalman_states.ox')
        lg_states.add_variable('kalman_states.oy')
        lg_states.add_variable('rssiCR.rssi')
        lg_states.add_variable('rssiCR.distance')

        lg_states.add_variable('rssiCR.pos_x')
        lg_states.add_variable('rssiCR.pos_y')

        fh = open("log_test.txt", "w")
    

        with SyncCrazyflie(self.URI, cf=cf) as scf:
            with MotionCommander(scf,0.8) as motion_commander:
                with MultiRanger(scf) as multi_ranger:
                    with Stabilization(scf) as stabilization:
                        with SyncLogger(scf, lg_states) as logger_states:
                            bug_controller = ComAngleLoopController()
                            bug_controller.init(0.7,0.5)


                            keep_flying = True
                            time.sleep(1)

                            param_name = "rssiCR.start"
                            param_value = "1"
                            cf.param.set_value(param_name, param_value)

                            twist.linear.x = 0.2
                            twist.linear.y = 0.0
                            twist.angular.z = 0
                            heading_prev = 0.0
                            heading = 0.0
                            angle_to_goal = 0.0;
                            kalman_x = 0.0
                            kalman_y = 0.0
                            already_reached_far_enough = False
                            state = "TURN_TO_GOAL"
                            prev_heading = stabilization.heading
                            angle_outbound = stabilization.heading;
                            state_start_time =0
                            rssi = 0
                            distance = 0
                            first_run = True
                            x_0 = 0
                            y_0 = 0
                            while keep_flying:

                                # crazyflie related stuff
                                for log_entry_1 in logger_states:
                                    data = log_entry_1[1]

                                    heading = math.radians(float(data["stabilizer.yaw"]));
                                    pos_x =-1*float(data["rssiCR.pos_x"])#float(data["kalman_states.ox"])-0.5
                                    pos_y =-1*float(data["rssiCR.pos_y"])#float(data["kalman_states.oy"])-1.5
                                    kalman_x = float(data["kalman_states.ox"])
                                    kalman_y = float(data["kalman_states.oy"])
                                    rssi = float(data["rssiCR.rssi"])
                                    distance = float(data["rssiCR.distance"])
                                    
                                    if first_run is True:
                                        x_0 = kalman_x
                                        y_0 = kalman_y
                                        first_run = False
                                        
                                    if pos_x == 0:
                                        pos_x = 0.02
                                    if pos_y == 0:
                                        pos_y = 0.02
                                        
                                        
                                    rel_coord_x = self.goal_coord_x[self.coord_index]-(kalman_x-x_0);
                                    rel_coord_y = self.goal_coord_y[self.coord_index]-(kalman_y-y_0);
                            
                                    self.distance_to_goal = math.sqrt(math.pow(rel_coord_x,2)+math.pow(rel_coord_y,2))
                                    self.angle_to_goal = wraptopi(np.arctan2(rel_coord_y,rel_coord_x))

                                    break


                                time.sleep(0.1)
                                
                                #Handle state transitions
                                if state =="STATE_MACHINE":
                                    if self.distance_to_goal<1.0   and self.coord_index is not len(self.goal_coord_x)-1:
                                        state = self.transition("TURN_TO_GOAL")
                                        self.coord_index = self.coord_index + 1
                                        self.distance_to_goal = 10
                                        self.angle_to_goal = 10
                                    if self.distance_to_goal<1.0  and self.coord_index is len(self.goal_coord_x)-1:
                                        state = self.transition("STOP")
                                if state =="TURN_TO_GOAL":
                                    if time.time()-self.state_start_time > 1 and logicIsCloseTo(stabilization.heading,wraptopi(self.angle_to_goal),0.1):
                                        state = self.transition("STATE_MACHINE")

                                #Handle state actions
                                if state=="STATE_MACHINE":
                                    twist = bug_controller.stateMachine(multi_ranger.front,multi_ranger.right,multi_ranger.left,stabilization.heading,wraptopi(self.angle_to_goal),self.distance_to_goal)
                                if state =="TURN_TO_GOAL":
                                    twist.linear.x = 0.0;
                                    twist.linear.y = 0.0;
                                    twist.angular.z = 0.6;
                                if state =="STOP":
                                    twist.linear.x = 0.0;
                                    twist.linear.y = 0.0;
                                    twist.angular.z = 0.0;
                                    keep_flying = False
                                if len(pos)>0:
                                    fh.write("%f, %f,  %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n"% (twist.linear.x, -1*math.degrees(twist.angular.z), self.distance_to_goal,self.angle_to_goal, stabilization.heading, kalman_x, kalman_y, pos_x, pos_y,pos[0],pos[2],rssi,distance))
                                else:
                                    fh.write("%f, %f,  %f, %f, %f, %f, %f, %f, %f, 0, 0, %f, %f\n"% (twist.linear.x, -1*math.degrees(twist.angular.z), self.distance_to_goal,self.angle_to_goal, stabilization.heading, kalman_x, kalman_y, pos_x, pos_y,rssi,distance))

                                print(state)
                                motion_commander._set_vel_setpoint(twist.linear.x,twist.linear.y,0,-1*math.degrees(twist.angular.z))

                                if multi_ranger.up is not None :
                                    if multi_ranger.up < 0.2:
                                        print("up range is activated")
                                        keep_flying = False

                            motion_commander.stop()

                            print("demo terminated")
                            fh.close()


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
