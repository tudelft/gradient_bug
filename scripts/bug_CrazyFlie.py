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
    if id == 3:
        pos = position
        #print( "Received frame for rigid body", id )

# Intialization of Optirack
streamingClient = NatNetClient()
streamingClient.newFrameListener = receiveNewFrame
streamingClient.rigidBodyListener = receiveRigidBodyFrame
streamingClient.run()

DISTANCE_TO_TRAVEL = 9.0
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
    URI = 'radio://0/40/250K'

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
        lg_states.add_variable('rssiCR.pos_x')
        lg_states.add_variable('rssiCR.pos_y')

        fh = open("log_test.txt", "w")

        with SyncCrazyflie(self.URI, cf=cf) as scf:
            with MotionCommander(scf,0.8) as motion_commander:
                with MultiRanger(scf) as multi_ranger:
                    with Stabilization(scf) as stabilization:
                        with SyncLogger(scf, lg_states) as logger_states:
                            bug_controller = ComController()
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
                            distance_to_goal = 0.0
                            state = "STATE_MACHINE"
                            prev_heading = stabilization.heading
                            angle_outbound = stabilization.heading;
                            state_start_time =0
                            while keep_flying:

                                # crazyflie related stuff
                                for log_entry_1 in logger_states:
                                    data = log_entry_1[1]

                                    heading = math.radians(float(data["stabilizer.yaw"]));
                                    pos_x =-1*float(data["rssiCR.pos_x"])#float(data["kalman_states.ox"])-0.5
                                    pos_y =-1*float(data["rssiCR.pos_y"])#float(data["kalman_states.oy"])-1.5
                                    kalman_x = float(data["kalman_states.ox"])-0.5
                                    kalman_y = float(data["kalman_states.oy"])-1.5

                                    if pos_x == 0:
                                        pos_x = 0.02
                                    if pos_y == 0:
                                        pos_y = 0.02
                                    if already_reached_far_enough:
                                        distance_to_goal = math.sqrt(math.pow(kalman_x,2) + math.pow(kalman_y,2))
                                        #angle_to_goal = wraptopi(np.pi+math.atan(pos_y/pos_x))
                                        angle_to_goal = wraptopi(np.pi+math.atan(kalman_y/kalman_x))
                                    else:
                                        distance_to_goal = DISTANCE_TO_TRAVEL - math.sqrt(math.pow(kalman_x,2) + math.pow(kalman_y,2))
                                        #angle_to_goal = wraptopi(math.atan(pos_y/pos_x))
                                        angle_to_goal = wraptopi(math.atan(kalman_y/kalman_x))

                                    break

                                if already_reached_far_enough:
                                    angle_goal = angle_to_goal
                                else:
                                    angle_goal = angle_outbound - 0.6

                                time.sleep(0.1)
                                print("stabilization",stabilization.heading)

                                if state =="STATE_MACHINE":
                                    print(distance_to_goal)
                                    print(already_reached_far_enough)
                                    if distance_to_goal<0.4  and already_reached_far_enough is False:
                                        state = self.transition("TURN_180")
                                        already_reached_far_enough = True
                                        distance_to_goal = 10
                                    if distance_to_goal<1.0  and already_reached_far_enough is True:
                                        keep_flying = False

                                if state =="TURN_180":
                                    if time.time()-self.state_start_time > 1 and logicIsCloseTo(stabilization.heading,wraptopi(angle_goal),0.1):
                                        state = self.transition("STATE_MACHINE")


                                if state=="STATE_MACHINE":
                                    twist = bug_controller.stateMachine(multi_ranger.front,multi_ranger.right,stabilization.heading,wraptopi(angle_goal),distance_to_goal)
                                if state =="TURN_180":
                                    twist.linear.x = 0.0;
                                    twist.linear.y = 0.0;
                                    twist.angular.z = 0.3;

                                    #OPTITRACK STUFF
                                if len(pos)>0:
                                    fh.write("%f, %f,  %f, %f, %f, %f, %f, %f, %f, %f, %f\n"% (twist.linear.x, -1*math.degrees(twist.angular.z), distance_to_goal,angle_to_goal, stabilization.heading, kalman_x, kalman_y, pos_x, pos_y,pos[0],pos[2]))
                                else:
                                    fh.write("%f, %f,  %f, %f, %f, %f, %f, %f, %f, 0, 0\n"% (twist.linear.x, -1*math.degrees(twist.angular.z), distance_to_goal,angle_to_goal, stabilization.heading, kalman_x, kalman_y, pos_x, pos_y))


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
