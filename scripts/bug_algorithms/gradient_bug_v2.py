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
from geometry_msgs.msg import PoseStamped


from sklearn import linear_model, datasets
import matplotlib.pyplot as plt
from scipy.signal import medfilt

from copy import deepcopy

import time
#import tf
import math
from _ast import IsNot

import numpy as np

from amcl.cfg.AMCLConfig import inf

def wraptopi(number):
    return  ( number + np.pi) % (2 * np.pi ) - np.pi


class GradientBugController:
    #First run of the algorithm
    first_run = True
    
    #For wall follower
    wall_follower = WallFollower()
    ref_distance_from_wall = 1.0
    already_reversed_direction=False
    direction = 1
    
    #For state transitions
    state_start_time = 0
    state = "FORWARD"
    state_WF =  ""

    #Max values
    max_speed = 0.2
    max_rate = 0.5
    
    # previous values
    heading_prev = 0.0
    prev_distance = 1000.0

    #For RSSI measurements
    do_circle = True
    angle_rssi = 0
    rssi_array=[]
    rssi_heading_array =[]
    rssi_goal_angle_adjust = 0;
    
    rssi_linear_array = []
    rssi_linear_array_max_size = 49
    
    prev_rssi = 0
    t = 0
    
    saved_pose = PoseStamped()
    saved_pose_hit = PoseStamped()
    overwrite_and_reverse_direction = False
    not_out_of_the_woods = True
    
    def init(self,new_ref_distance_from_wall,max_speed_ref = 0.2, max_rate_ref = 0.5):
        self.ref_distance_from_wall = new_ref_distance_from_wall
        self.state = "ROTATE_TO_GOAL"
        self.max_speed = max_speed_ref
        self.max_rate = max_rate_ref
        self.rssi_goal_angle_adjust = 0
        self.first_run = True
        self.do_circle = True
        self.rssi_array = []
        self.rssi_heading_array = []
        self.rssi_linear_array = []
        self.state_start_time = 0
        self.angle_rssi=0
        self.heading_prev = 0
        self.prev_distance = 1000
        self.already_reversed_direction=False
        self.direction = 1
        self.saved_pose = PoseStamped()
        self.saved_pose_hit = PoseStamped()

        self.overwrite_and_reverse_direction = False
        self.not_out_of_the_woods = True
        self.loop_angle=0

    def take_off(self):
        twist = Twist()
        twist.linear.z = 0.1;
        return twist

    def hover(self):
        twist = Twist()
        return twist


    def twistForward(self):
        v = self.max_speed
        w = 0
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist
    
    def twistTurn(self, rate):
        v = 0
        w = rate
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist
    
    def twistTurnCircle(self, radius):
        v = self.max_speed
        w = self.direction*(-v/radius)
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist
    
    
    def twistForwardAlongWall(self, range):
        twist = Twist()
        twist.linear.x = self.max_speed
        if  self.logicIsCloseTo(self.ref_distance_from_wall, range, 0.1) == False:
            if range>self.ref_distance_from_wall:
                twist.linear.y = self.direction*( - self.max_speed/3)
            else:
                twist.linear.y = self.direction*self.max_speed /3

        return twist
    
        



    def logicIsCloseTo(self,real_value = 0.0, checked_value =0.0, margin=0.05):

        if real_value> checked_value-margin and real_value< checked_value+margin:
            return True
        else:
            return False

    # Transition state and restart the timer
    def transition(self, newState):
        state = newState
        self.state_start_time = self.simulator_time
        return state
    
    


        

    def stateMachine(self, front_range, right_range, left_range, current_heading, bearing_goal, distance_goal, rssi_to_tower,odometry, correct_time, from_gazebo = True, WF_argos = None, RRT= None, outbound = False, bot_is_close = False):
        
        
        #Initialization of the twist command
        twist = Twist()
        
        #Save simulator time as the correct time
        self.simulator_time = correct_time;

        # Deal with none values of the range sensors
        if front_range == None:
            front_range = inf
        if right_range == None:
            right_range = inf
        if left_range == None:
            left_range = inf
 
        # First thing to take care of at the very first run
        if self.first_run is True:
            self.prev_distance = 2000;#distance_goal;
            self.angle_rssi = 2000;
            self.first_run = False
            self.heading_prev=0#current_heading
            self.state_start_time = correct_time
            
        

        # Bearing to goal is the angle to goal
        #TODO check if this is also correct for the gazebo implementation....

        #################### STATE TRANSITIONS#####################
        # Forward
        if self.state == "FORWARD":
 
            #If front range is activated to be close et the other wall    
            if front_range < self.ref_distance_from_wall+0.2:
            
                
                #Initialize the wallfollower
                if from_gazebo:
                    self.wall_follower.init(self.ref_distance_from_wall,self.max_speed)
                else:
                    WF_argos.init()
                    
                    
                # save previous heading and initialize the reverse option
                self.heading_prev = deepcopy(current_heading)

                # First check if the bug has not gotten himself in a loop
                if self.overwrite_and_reverse_direction:
                    self.direction = -1*self.direction
                    self.overwrite_and_reverse_direction = False
                else:
                   # To evaluate the wall angle for the local direction (replace scan_obstacle)
                    if left_range<right_range and left_range < 2.0:
                        self.direction = -1
                    elif left_range>right_range and right_range < 2.0:
                        self.direction = 1
                    elif left_range>2.0 and right_range>2.0:
                        self.direction = 1
                    else:
                        self.direction = 1
                    
                #Save the hitpoint for loop checking
                self.saved_pose_hit = deepcopy(odometry)

                #Go to wall_following
                self.state = self.transition("WALL_FOLLOWING")
                self.already_reversed_direction = False
                pass

        # Wall Following
        elif self.state == "WALL_FOLLOWING":
            
            #Detect the relative position of the robot and the hitpoint to dietect loops
            rel_x_loop = odometry.pose.position.x- self.saved_pose_hit.pose.position.x 
            rel_y_loop = odometry.pose.position.y - self.saved_pose_hit.pose.position.y  
            
            temp_loop_angle =  wraptopi(np.arctan2(rel_y_loop,rel_x_loop))
            self.loop_angle =float(temp_loop_angle[0])
            
            if bot_is_close and self.already_reversed_direction is False:
                self.already_reversed_direction=True
                self.state = self.transition("REVERSE_DIRECTION")
                pass

            
            # If it is rotating around a wall, front range is free and it is close to the angle_goal
            if self.state_WF is "ROTATE_AROUND_WALL" or self.state_WF is "ROTATE_AROUND_CORNER":
                if front_range>1.5 and (abs(wraptopi(bearing_goal))<0.2): #version 2
                
                    # Check if the robot has moved behing him
                    if (abs(wraptopi(current_heading+bearing_goal+np.pi-self.loop_angle))<0.5):
                        self.overwrite_and_reverse_direction = True
                        #print("LOOPING!")
                    
                    self.saved_pose = deepcopy(odometry) 

                    #Save previous distance for reverse direction possibility
                    # Goto rotate_to_goal
                    self.state = self.transition("ROTATE_TO_GOAL")
                    pass
            # If the previous saved distance is smaller than the current one and it hasn't reverse direction yet
            
        # Rotate to Goal
        elif self.state=="ROTATE_TO_GOAL": 
            # If the heading is close to the angle goal   
            if( self.logicIsCloseTo(bearing_goal,0,0.2)):
                self.state = self.transition("FORWARD")
        # Reverse (local) direction
        elif self.state =="REVERSE_DIRECTION":
            # if the front range sensor is activated, go back to wall_following in the other direction
            if front_range < self.ref_distance_from_wall+0.2:
                # Reverse local direction flag
                if self.direction == -1:
                    self.direction = 1
                elif self.direction == 1:
                    self.direction = -1
                if from_gazebo:
                    self.wall_follower.init(self.ref_distance_from_wall,self.max_speed)
                else:
                    WF_argos.init()
                #Go to wall_following
                self.state = self.transition("WALL_FOLLOWING")
                pass
     
            
            #For debugging in matlab, uncomment this!
             #   np.savetxt('plot_rssi_array.txt',self.rssi_array,delimiter=',')
             #   np.savetxt('plot_rssi_heading_array.txt',self.rssi_heading_array,delimiter=',')
             #   np.savetxt('plot_angle_rssi.txt',[self.angle_rssi, self.rssi_goal_angle_adjust])


        #print(self.state)

        #################### STATE ACTIONS #####################
        #Forward
        if self.state == "FORWARD":
            #Go forward
            twist=self.twistForward()
            
            if from_gazebo:
                # If the left or right range is activated during forward, move it away from the wall
                #TODO: find out if this is still necessary
                if(left_range<self.ref_distance_from_wall):
                    twist.linear.y = twist.linear.y-0.2;
                if(right_range<self.ref_distance_from_wall):
                    twist.linear.y = twist.linear.y+0.2;
        # Wall_follwoing
        elif self.state == "WALL_FOLLOWING":
            # Use the wallfollowing controller, unique per simulated robot
            if from_gazebo is True:
                if self.direction is 1:
                    twist, self.state_WF = self.wall_follower.wall_follower(front_range,right_range, current_heading,self.direction)
                else:
                    twist, self.state_WF = self.wall_follower.wall_follower(front_range,left_range, current_heading,self.direction)
            else:
                if self.direction is 1:
                    twist, self.state_WF = WF_argos.wallFollowingController(RRT.getRangeRight(),RRT.getRangeFrontRight(), RRT.getLowestValue(),RRT.getHeading(),RRT.getArgosTime(),self.direction)
                else:
                    twist, self.state_WF = WF_argos.wallFollowingController(RRT.getRangeLeft(),RRT.getRangeFrontLeft(), RRT.getLowestValue(),RRT.getHeading(),RRT.getArgosTime(),self.direction)
                # Reverse the heading command 
                #TODO check this out why this is the case
                twist.angular.z = twist.angular.z*-1
        #Rotate to goal
        elif self.state=="ROTATE_TO_GOAL":
            # To make sure that the robot is turning in the right direction to sae time
            if (self.angle_rssi-current_heading)>0:
                twist = self.twistTurn(self.max_rate)
            else:
                twist = self.twistTurn(-1*self.max_rate)
        if self.state =="REVERSE_DIRECTION":
            # Just turn towards the wall
            twist = self.twistTurn(self.direction*-0.5)
    
        
        #return twist and the adjusted angle goal by the rssi            
        return twist, self.rssi_goal_angle_adjust



if __name__ == '__main__':
    try:
        wall_follower()
    except rospy.ROSInterruptException:
        pass
