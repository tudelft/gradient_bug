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


from sklearn import linear_model, datasets
import matplotlib.pyplot as plt
from scipy.signal import medfilt


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
    do_circle = False
    angle_rssi = 0
    rssi_array=[]
    rssi_heading_array =[]
    rssi_goal_angle_adjust = 0;

    
    def init(self,new_ref_distance_from_wall,max_speed_ref = 0.2, max_rate_ref = 0.5):
        self.ref_distance_from_wall = new_ref_distance_from_wall
        self.state = "ROTATE_TO_GOAL"
        self.max_speed = max_speed_ref
        self.max_rate = max_rate_ref
        self.rssi_goal_angle_adjust = 0
        self.first_run = True


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
    
    


        

    def stateMachine(self, front_range, right_range, left_range, current_heading, angle_goal, distance_goal, rssi_to_tower, correct_time, from_gazebo = True, WF_argos = None, RRT= None):
        
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
            self.prev_distance = distance_goal;
            self.angle_rssi = angle_goal
            self.first_run = False

        # Bearing to goal is the angle to goal
        #TODO check if this is also correct for the gazebo implementation....
        bearing = angle_goal;
        bearing_with_adjust = angle_goal #self.rssi_goal_angle_adjust;
        self.heading = current_heading;

        print("before", self.state)
        print(bearing_with_adjust)
        print(self.rssi_goal_angle_adjust)
        

        #################### STATE TRANSITIONS#####################
        # Forward
        if self.state == "FORWARD":
            #If need to do circle and 1 second has passed
            if self.do_circle == True and correct_time-self.state_start_time > 1:
                # Initialize rssi and heading arrays and save previous heading
                self.rssi_array = []
                self.rssi_heading_array = []
                self.heading_prev=current_heading
                # Go to rotate_360
                self.state = self.transition("ROTATE_360")
            #If front range is activated to be close et the other wall    
            if front_range < self.ref_distance_from_wall+0.2:
                #Initialize the wallfollower
                if from_gazebo:
                    self.wall_follower.init(self.ref_distance_from_wall,self.max_speed)
                else:
                    WF_argos.init()
                # save previous heading and initialize the reverse option
                self.heading_prev = current_heading
                self.already_reversed_direction = False
                # To evaluate the wall angle for the local direction (replace scan_obstacle)
                if left_range<right_range and left_range < 2.0:
                    self.direction = -1
                if left_range>right_range and right_range < 2.0:
                    self.direction = 1
                if left_range>2.0 and right_range>2.0:
                    self.direction = 1
                #Go to wall_following
                self.state = self.transition("WALL_FOLLOWING")
        # Reverse (local) direction
        elif self.state =="REVERSE_DIRECTION":
            # if the front range sensor is activated, go back to wall_following in the other direction
            if front_range < self.ref_distance_from_wall+0.2:
                # Reverse local direction flag
                if self.direction == -1:
                    self.wall_angle = -1
                elif self.direction == 1:
                    self.wall_angle = 1
                #Go to wall_following
                self.state = self.transition("WALL_FOLLOWING")
        # Wall Following
        elif self.state == "WALL_FOLLOWING":
            # If it is rotating around a wall, front range is free and it is close to the angle_goal
            if self.state_WF is "ROTATE_AROUND_WALL" or self.state_WF is "ROTATE_AROUND_CORNER":
                if front_range>1.5 and (bearing_with_adjust>-0.2 and bearing_with_adjust < 0.2):
                    # Indicate that the rssi finding circle needs to be made
                    self.do_circle = True
                    # Goto rotate_to_goal
                    self.state = self.transition("ROTATE_TO_GOAL")
            # If the previous saved distance is smaller than the current one and it hasn't reverse direction yet
            if self.prev_distance<distance_goal and self.already_reversed_direction is False:
                # Already reversed direction to prevent it from happinening again during the wallfolowing
                self.already_reversed_direction = True
                # Go to reverse_direction
                self.state = self.transition("REVERSE_DIRECTION")
        # Rotate to Goal
        elif self.state=="ROTATE_TO_GOAL": 
            # If the heading is close to the angle goal           
            if self.logicIsCloseTo(bearing_with_adjust,0,0.1):
                #Go to forward
                self.state = self.transition("FORWARD")
        #Rotate 360
        elif self.state=="ROTATE_360":
            # if 2 seconds has passed, the previous heading is close to the current heading and do_circle flag is on
            if correct_time-self.state_start_time > 2 and self.logicIsCloseTo(current_heading,wraptopi( self.heading_prev),0.1) and self.do_circle:
                #do_circle flag is on false so it knows it is finished
                self.do_circle = False
                #Filter the saved rssi array
                rssi_array_filt = medfilt(self.rssi_array,9)
                #Find the maximum RSSI and it's index               
                index_max_rssi =np.argmax(rssi_array_filt)
                # Retrieve the offset angle to the goal
                self.angle_rssi =wraptopi(self.rssi_heading_array[index_max_rssi]+3.14)
                # Determine the adjusted goal angle, which is added to the heading later
                self.rssi_goal_angle_adjust = wraptopi(current_heading-self.angle_rssi)
                # Go to rotate to goal
                self.state = self.transition("ROTATE_TO_GOAL")
                np.savetxt('plot_rssi_array.txt',self.rssi_array,delimiter=',')
                np.savetxt('plot_rssi_heading_array.txt',self.rssi_heading_array,delimiter=',')




        print("after",self.state)

        

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
        # Reverse direction
        if self.state =="REVERSE_DIRECTION":
            # Just turn towards the wall
            twist = self.twistTurn(self.direction*-0.5)
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
            if bearing_with_adjust>0:
                twist = self.twistTurn(self.max_rate)
            else:
                twist = self.twistTurn(-1*self.max_rate)
        # Rotate 360 degrees
        elif self.state =="ROTATE_360":
            # If do_circle flag is on, save the rssi and angle goal in a array
            if self.do_circle is True:
                self.rssi_array.append(rssi_to_tower)
                self.rssi_heading_array.append(current_heading)
            #Turn with max_rate
            twist = self.twistTurn(self.max_rate*0.5)
            
            

                    
        return twist, self.rssi_goal_angle_adjust



if __name__ == '__main__':
    try:
        wall_follower()
    except rospy.ROSInterruptException:
        pass
