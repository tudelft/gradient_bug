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

import random

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
    state = "ROTATE_TO_GOAL"
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
    
    angle_goal = 0
    goal_angle_dir = -1
    correct_heading_array = []
    rssi_sample_reset =  False
    prev_rssi = 0;
    saved_pose_sample = PoseStamped()
    
    def init(self,new_ref_distance_from_wall,max_speed_ref = 0.2, max_rate_ref = 0.5, angle_goal_init=-2.27):
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
        self.angle_goal = angle_goal_init
        self.goal_angle_dir = -1
        
        self.correct_heading_array = [0, 0, 0, 0, 0, 0, 0, 0]
        self.rssi_sample_reset = False

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
    
    
    def fillHeadingArray(self,rssi_heading, diff_rssi, max_meters):
        
        #heading array of action choices
        heading_array = [-135.0, -90.0, -45.0, 0.0, 45.0, 90.0, 135.0, 180.0]
        rssi_heading_deg = math.degrees(rssi_heading);
        #print(rssi_heading_deg)
        
        for it in range(8):
            #fill array based on heading and rssi heading
            if ((rssi_heading_deg>=heading_array[it]-22.5) and (rssi_heading_deg<heading_array[it]+22.5) and it is not 7) or \
            (it is 7 and ((rssi_heading_deg>=heading_array[it]-22.5) or (rssi_heading_deg<-135.0 - 22.5))):
                
                temp_value_forward = self.correct_heading_array[it]
                temp_value_backward =  self.correct_heading_array[(it + 4) % 8]
                
                #If gradient is good, increment the array corresponding to the current heading and
                #   decrement the exact opposite
                if (diff_rssi> 0):
                    self.correct_heading_array[it]=temp_value_forward + 1
                    if (temp_value_backward >0):
                        self.correct_heading_array[(it+4)%8]=temp_value_backward - 1
                # if gradient is bad, decrement the array corresponding to the current heading and 
                #   increment the exact opposite
                elif diff_rssi<0:
                    self.correct_heading_array[(it+4)%8] = temp_value_backward + 1
                    if(temp_value_forward>0):
                        self.correct_heading_array[it] = temp_value_forward - 1
                        
                        
        # degrading function
        #    If one of the arrays goes over maximum amount of points (meters), then decrement all values                  
                        
        if max(self.correct_heading_array)>max_meters:
            for it in range(8):
                if self.correct_heading_array[it]>0:
                    self.correct_heading_array[it]=self.correct_heading_array[it]-1
                    
                    
        # Calculate heading where the beacon might be
        count = 0
        y_part = 0
        x_part = 0
        
        for it in range(8):
            if(self.correct_heading_array[it]>0):
                x_part += self.correct_heading_array[it]*math.cos(heading_array[it]*math.pi/180.0)
                y_part += self.correct_heading_array[it]*math.sin(heading_array[it]*math.pi/180.0)
                count = count + self.correct_heading_array[it]
                
        wanted_angle_return = 0
        if count is not 0:
            wanted_angle_return = math.atan2(y_part/count,x_part/count)
    
    
        #print("array",self.correct_heading_array)
        #print("array",wanted_angle_return)
        return wanted_angle_return

        
    


        

    def stateMachine(self, front_range, right_range, left_range, current_heading, distance_goal,
                      rssi_to_tower,odometry, correct_time, from_gazebo = True, WF_argos = None,
                       RRT= None, outbound = False, distance_other_bot = 5, priority=False):
        
        
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
            
            
        bot_is_close = False
        bot_is_really_close = False
        if distance_other_bot < 2:
            
            bot_is_close = True
            if distance_other_bot<1:
                bot_is_really_close = True
        

        # Bearing to goal is the angle to goal
        #TODO check if this is also correct for the gazebo implementation....

        #################### STATE DEFINITIONS ####################
        
        # FORWARD
        # WALLFOLLOWING
        # ROTATE_TO_GOAL
        # MOVE_OUT_OF_WAY

        #print self.overwrite_and_reverse_direction
        #print self.state
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
                    rand_num = random.randint(1,101)
                    if left_range<right_range and left_range < 2.0:
                        self.direction = -1

                    elif left_range>right_range and right_range < 2.0:
                        self.direction = 1

                    elif left_range>2.0 and right_range>2.0:
                            self.direction = 1
                    else:
                            self.direction = -1  
                    
                #Save the hitpoint for loop checking
                self.saved_pose_hit = deepcopy(odometry)


                # GRADIENTBUG: implement reinitialize heading array!!
                self.correct_heading_array = [0,0,0,0,0,0,0,0]
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
            temp_loop_distance = np.sqrt(rel_x_loop*rel_x_loop+rel_y_loop*rel_y_loop)
            self.loop_angle =float(temp_loop_angle[0])
            
             # Check if the robot has moved behing him
            if (abs(wraptopi(self.angle_goal+np.pi-self.loop_angle))<1.0)and temp_loop_distance>1:
                self.overwrite_and_reverse_direction = True
                
                        
            # if another bot is close and priority is lower than the other bot, stop moving
            if bot_is_close and priority is False:
                #GRADIENTBUG: See if bearing goal needs flipping or something else
                if outbound:
                    #if (other_goal_angle < 0 and self.angle_goal < 0) or (other_goal_angle>0 and self.angle_goal>0):
                     #  self.angle_goal = -1* self.angle_goal
                     #  self.goal_angle_dir = wraptopi(current_heading-self.angle_goal)
                    self.angle_goal = self.angle_goal
                if bot_is_really_close:
                    self.state = self.transition("MOVE_OUT_OF_WAY")
                pass

            bearing_to_goal = wraptopi(self.angle_goal - current_heading)
            goal_accesability_check = False;
            if (self.direction == -1):
                goal_accesability_check = bearing_to_goal<0 and bearing_to_goal>-1.57
            else:
                goal_accesability_check = bearing_to_goal>0 and bearing_to_goal<1.57

            # If it is rotating around a wall, front range is free and it is close to the angle_goal
            if self.state_WF is "ROTATE_AROUND_WALL" or self.state_WF is "ROTATE_AROUND_CORNER":
                if front_range>1.5 and goal_accesability_check: 
                                    
                    self.saved_pose = deepcopy(odometry) 
                    self.goal_angle_dir = wraptopi(current_heading-self.angle_goal)
                    # Goto rotate_to_goal
                    self.state = self.transition("ROTATE_TO_GOAL")
                    pass
            
            if self.state_WF is "WALL_FOLLOWING":
                #reset sample gattering
                if self.rssi_sample_reset:
                    self.saved_pose_sample = deepcopy(odometry)
                    self.rssi_sample_reset = False
                    self.prev_rssi = rssi_to_tower
                
                rel_x_sample = odometry.pose.position.x- self.saved_pose_sample.pose.position.x 
                rel_y_sample = odometry.pose.position.y - self.saved_pose_sample.pose.position.y  
                distance = math.sqrt(rel_x_sample*rel_x_sample + rel_y_sample*rel_y_sample)
                
                if distance > 1.0:
                    self.rssi_sample_reset = True
                    heading_rssi = current_heading
                    diff_rssi =  self.prev_rssi - rssi_to_tower
                    #print("diff rssi", diff_rssi)
                    
                    if outbound is False:
                        self.angle_goal = self.fillHeadingArray(heading_rssi, diff_rssi, 5)
            #GRADIENT implement the gradient search during wallfollowing here!!
            
        # Rotate to Goal
        elif self.state=="ROTATE_TO_GOAL": 
            # If the heading is close to the angle goal
            goal_check = self.logicIsCloseTo(current_heading-self.angle_goal, 0.0, 0.1)   
            if goal_check:
                self.state = self.transition("FORWARD")
        # Reverse (local) direction
        elif self.state =="MOVE_OUT_OF_WAY":
            if bot_is_really_close is False:
                #GRotate to goal
                self.state = self.transition("ROTATE_TO_GOAL")
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
            if self.goal_angle_dir<0:
                twist = self.twistTurn(self.max_rate)
            else:
                twist = self.twistTurn(-1*self.max_rate)
        if self.state =="MOVE_OUT_OF_WAY":
            
            if from_gazebo is True:
                #TODO implement true move out of way for gazebo, just like the real quadcopter
                twist = self.hover()
            else:
                # In argos, just let the other bot go past you, no need to move out of way 
                twist = self.hover()
    
        
        #return twist and the adjusted angle goal by the rssi            
        return twist, self.rssi_goal_angle_adjust



if __name__ == '__main__':
    try:
        wall_follower()
    except rospy.ROSInterruptException:
        pass
