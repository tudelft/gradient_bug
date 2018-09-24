#!/usr/bin/env python
# license removed for brevity
import rospy
import sys
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from hector_uav_msgs.srv import EnableMotors
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from rosgraph_msgs.msg import Clock

from std_msgs.msg import Float32


import matplotlib.pyplot as plt

from std_srvs.srv import Trigger
#from wall_follower_multi_ranger import WallFollower
from std_srvs.srv import Empty

sys.path.append('/home/knmcguire/Software/catkin_ws/src/gradient_bug/scripts/bug_algorithms')

from wall_following_controller import WallFollowerController
from com_controller import ComController
from com_angle_controller import ComAngleController
from com_angle_loop_controller import ComAngleLoopController
from gradient_bug_v1 import GradientBugController

import time
import tf
import math
from _ast import IsNot

import numpy as np
def logicIsCloseTo( real_value = 0.0, checked_value =0.0, margin=0.05):

    if real_value> checked_value-margin and real_value< checked_value+margin:
        return True
    else:
        return False


def wraptopi(number):
    return  ( number + np.pi) % (2 * np.pi ) - np.pi


class bug_gazebo:
    # Callbacks
    front_range = 0.0
    right_range = 0.0
    left_range = 0.0

    altitude = 0.0
    state = "TAKE_OFF"
    current_heading = 0.0
    distance_to_goal = 0.0
    
    distance_to_goal_drift = 0.0
    angle_to_goal_drift = 0.0
    
    angle_to_goal =  0.0
    already_reached_far_enough = False
    DISTANCE_TO_TRAVEL = 10.0
    angle_outbound = 0.0
    state_start_time = 0.0
    
    goal_coord_x = [4]
    goal_coord_y = [4]
    
    coord_index = 0
    
    odometry = PoseStamped()
    
    start_position_x=0
    start_position_y=0
    
    
    groundtruth_pose = PoseStamped()
    
    first_run = True
    prev_time = 0.0
    
    
    first_heading = 0.0;
    current_heading_drift= 0.0;
    
    gazebo_time = 0
    
    distance_from_wall = 0.5
    
    rssi_to_tower = 0

    
    def get_odometry_from_commands(self,noisy_command):
        
        
        current_time = self.gazebo_time
        
        diff_time = current_time-self.prev_time
        self.prev_time = current_time

        

        noisy_velocity_estimate_x = noisy_command.linear.x;
        noisy_velocity_estimate_y = noisy_command.linear.y;

        self.current_heading_drift = wraptopi(self.current_heading_drift + diff_time*noisy_command.angular.z)
       # self.current_heading_drift = self.current_heading+0.2
        self.odometry.pose.position.x = self.odometry.pose.position.x + diff_time*(noisy_velocity_estimate_x*math.cos(self.current_heading_drift) - noisy_velocity_estimate_y*math.sin(self.current_heading_drift))
        self.odometry.pose.position.y = self.odometry.pose.position.y + diff_time*(noisy_velocity_estimate_x*math.sin(self.current_heading_drift) + noisy_velocity_estimate_y*math.cos(self.current_heading_drift))
        
        rel_coord_x = self.goal_coord_x[self.coord_index]-self.odometry.pose.position.x;
        rel_coord_y = self.goal_coord_y[self.coord_index]-self.odometry.pose.position.y;
        
        #print("rel coord odo",rel_coord_x,rel_coord_y )

        self.distance_to_goal_drift = math.sqrt(math.pow(rel_coord_x,2)+math.pow(rel_coord_y,2))

        self.angle_to_goal_drift = wraptopi(np.arctan2(rel_coord_y,rel_coord_x))
        
        return self.odometry
    
    
    def adjusted_odometry(self,angle_adjusted,distance_goal):
        odometry_adjusted = PoseStamped()
        odometry_adjusted.pose.position.x = self.odometry.pose.position.x*math.cos(angle_adjusted)- self.odometry.pose.position.y*math.sin(angle_adjusted)
        odometry_adjusted.pose.position.y = self.odometry.pose.position.x*math.sin(angle_adjusted)+ self.odometry.pose.position.y*math.cos(angle_adjusted)
        return odometry_adjusted



    def poseCB(self,state):
        
        if (self.first_run):
            self.start_position_x = state.pose.position.x
            self.start_position_y = state.pose.position.y
            

        
        self.groundtruth_pose = state
        
        #index_drone = len(state.pose)-1
        self.altitude = state.pose.position.z

        rel_coord_x = self.goal_coord_x[self.coord_index]-state.pose.position.x;
        rel_coord_y = self.goal_coord_y[self.coord_index]-state.pose.position.y;
       # print("rel coord pose",rel_coord_x,rel_coord_y )

        self.distance_to_goal = math.sqrt(math.pow(rel_coord_x,2)+math.pow(rel_coord_y,2))

        self.angle_to_goal = wraptopi(np.arctan2(rel_coord_y,rel_coord_x))



    def imuCB(self,imu):
        explicit_quat = [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w]

        euler = tf.transformations.euler_from_quaternion(explicit_quat)
        self.current_heading = euler[2]
        
        
    def towerRSSICB(self,data):
        
        self.rssi_to_tower = float(data.data)



    def frontRangeCB(self,range):
        self.front_range = min(range.ranges)

    def rightRangeCB(self,range):
        self.right_range = min(range.ranges)
        
    def leftRangeCB(self,range):
        self.left_range = min(range.ranges)
        
    def clockCB(self,clock):
        
        self.gazebo_time = float(clock.clock.secs)+float(clock.clock.nsecs)/1000000000.0
        #print("gazebo time",self.gazebo_time)

    def noisyTwist(self, twist, std_vel, std_rate):
        noisy_twist = Twist()
        noisy_twist.linear.x =np.random.normal(twist.linear.x,std_vel,1);
        noisy_twist.linear.y = np.random.normal(twist.linear.y,std_vel,1);
        noisy_twist.angular.z = np.random.normal(twist.angular.z,std_rate,1);
        noisy_twist.linear.z= twist.linear.z;
        return noisy_twist
            # Transition state and restart the timer
    def transition(self, newState):
        state = newState
        self.state_start_time = self.gazebo_time
        return state

    def init(self):
        rospy.init_node('wall_follower_multirange', anonymous=True)
        rospy.sleep(3.)
        rospy.Subscriber("ground_truth_to_tf/pose", PoseStamped, self.poseCB)
        rospy.Subscriber("raw_imu", Imu, self.imuCB)
        rospy.Subscriber("multi_ranger/front_range_sensor_value", LaserScan, self.frontRangeCB)
        rospy.Subscriber("multi_ranger/right_range_sensor_value", LaserScan, self.rightRangeCB)
        rospy.Subscriber("multi_ranger/left_range_sensor_value", LaserScan, self.leftRangeCB)
        
        
        rospy.Subscriber("/clock",Clock,self.clockCB)
        
        rospy.Subscriber("RSSI_to_tower",Float32,self.towerRSSICB)

        rospy.sleep(1.)

        rospy.wait_for_service('enable_motors')
        enable_motors  = rospy.ServiceProxy('enable_motors', EnableMotors)
        enable_motors(True)
        
    

    def rosloop(self):
        bug_controller = GradientBugController()
        #bug_controller = WallFollowerController()


        self.distance_from_wall =0.7

        bug_controller.init(self.distance_from_wall,0.5)
        twist = Twist()
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.state = "TAKE_OFF"
        rate = rospy.Rate(30) # 10hz

        prev_heading = 0.0
        
        rospy.wait_for_service('/indoor_gen')
        indoor_gen = rospy.ServiceProxy('/indoor_gen',Trigger)
        indoor_gen()
        
        
        spawn_env = rospy.ServiceProxy('/spawn_generated_environment',Empty)
        spawn_env()
        
        
        
        
        '''plt.ion()
        fig = plt.figure()
        plt.ylim(-10,10)
        plt.xlim(-10,10)
        '''
        
        it_plot=0
        
        
        fh = open("log.txt", "w")
        fh.write("pos_x, pos_y, pos_x_drift, pos_y_drift, pos_x_corrected, pos_y_corrected, angle_adjust\n")
        

        angle_adjust = 0
        while not rospy.is_shutdown():
        
            if self.first_run:
                self.odometry.pose.position.x =self.start_position_x;
                self.odometry.pose.position.y =self.start_position_y;
                self.first_heading = self.current_heading
                self.first_run = False
                enable_motors  = rospy.ServiceProxy('enable_motors', EnableMotors)
                enable_motors(True)
                
                
            



            if self.state == "TAKE_OFF":
                if self.altitude > 0.5:
                    self.state = self.transition("TURN_TO_GOAL")
                    self.angle_outbound = self.current_heading;
            if self.state =="STATE_MACHINE":
                if self.distance_to_goal<0.5 and self.coord_index is not len(self.goal_coord_x)-1:
                    self.state = self.transition("TURN_TO_GOAL")
                    self.coord_index = self.coord_index + 1
                #Put distance and angle to goal to an high number to prevent it from stopping right away
                    self.distance_to_goal = 10
                    self.angle_to_goal = 10
                    prev_heading = self.current_heading
                #if self.distance_to_goal<1.0 and self.coord_index is len(self.goal_coord_x)-1:
                if self.distance_to_goal<1.0 and self.coord_index is len(self.goal_coord_x)-1:
                    self.state = self.transition("STOP")
            if self.state =="TURN_TO_GOAL":
                if time.time()-self.state_start_time > 1 and logicIsCloseTo(self.current_heading,wraptopi(self.angle_to_goal),0.1):
                    self.state = self.transition("STATE_MACHINE")
            
            
            

            if self.state == "TAKE_OFF":
                twist.linear.z = 0.1;
            if self.state =="STATE_MACHINE":

                #twist = bug_controller.stateMachine(self.front_range,self.right_range, self.left_range, self.current_heading, wraptopi(self.angle_to_goal), self.distance_to_goal)
                twist, angle_adjust = bug_controller.stateMachine(self.front_range,self.right_range, self.left_range, self.current_heading_drift, wraptopi(self.angle_to_goal_drift-self.current_heading_drift), self.distance_to_goal_drift,self.rssi_to_tower,self.gazebo_time)
                #twist = bug_controller.stateMachine(self.front_range,self.right_range, self.left_range, wraptopi(self.current_heading), wraptopi(self.angle_to_goal_drift), self.distance_to_goal_drift,self.rssi_to_tower)

                
            if self.state =="TURN_TO_GOAL":
                twist.linear.x = 0.0;
                twist.linear.y = 0.0;
                twist.linear.z = 0.0;
                twist.angular.z = 0.3;
            if self.state =="STOP":
                twist.linear.x = 0.0;
                twist.linear.y = 0.0;
                twist.angular.z = 0.0;

            
                    
            

            
            noisy_twist = self.noisyTwist(twist,0.01,0.002);
            if self.first_run is False:
                self.get_odometry_from_commands(twist)
                

             

            '''       
            if it_plot > 100:
                plt.plot(self.groundtruth_pose.pose.position.x,self.groundtruth_pose.pose.position.y, 'go')
                plt.hold(True)
    
                plt.plot(self.odometry.pose.position.x,self.odometry.pose.position.y, 'ro')
    
                plt.hold(True)
                
                #plt.plot(self.gazebo_time,self.rssi_to_tower,'ro')
               # plt.hold(True)

                fig.canvas.draw()
                it_plot = 0
            else:
                it_plot=it_plot+1
            '''

            odometry_adjusted = self.adjusted_odometry(angle_adjust,self.distance_to_goal_drift)
            
            #plt.show(block = False)
                
            #print("heading", self.current_heading, self.current_heading_drift)
            #print("angle goal", self.angle_to_goal, self.angle_to_goal_drift)
            #print("angle distance", self.distance_to_goal, self.distance_to_goal_drift)

            fh.write("%f, %f,  %f, %f, %f, %f, %f\n"% (self.groundtruth_pose.pose.position.x,self.groundtruth_pose.pose.position.y,self.odometry.pose.position.x,self.odometry.pose.position.y,odometry_adjusted.pose.position.x,odometry_adjusted.pose.position.y,angle_adjust ))

            
            print self.state
            pub.publish(noisy_twist)

            rate.sleep()
            

            

if __name__ == '__main__':
    
    
    

    bug_gazebo = bug_gazebo()
    bug_gazebo.init()
    try:
        bug_gazebo.rosloop()
    except rospy.ROSInterruptException:
        pass
