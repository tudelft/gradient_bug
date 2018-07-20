#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from hector_uav_msgs.srv import EnableMotors
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
#from wall_follower_multi_ranger import WallFollower
from wall_following_controller import WallFollowerController
from com_controller import ComController
from com_angle_controller import ComAngleController
from com_angle_loop_controller import ComAngleLoopController

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
    angle_to_goal =  0.0
    already_reached_far_enough = False
    DISTANCE_TO_TRAVEL = 10.0
    angle_outbound = 0.0
    state_start_time = 0.0
    
    goal_coord_x = [8, 0]
    goal_coord_y = [2, 0]
    
    coord_index = 0


    def poseCB(self,state):

        index_drone = len(state.pose)-1
        self.altitude=state.pose[index_drone].position.z

        rel_coord_x = self.goal_coord_x[self.coord_index]-state.pose[index_drone].position.x;
        rel_coord_y = self.goal_coord_y[self.coord_index]-state.pose[index_drone].position.y;

        self.distance_to_goal = math.sqrt(math.pow(rel_coord_x,2)+math.pow(rel_coord_y,2))

        self.angle_to_goal = wraptopi(np.arctan2(rel_coord_y,rel_coord_x))



    def imuCB(self,imu):
        explicit_quat = [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w]

        euler = tf.transformations.euler_from_quaternion(explicit_quat)
        self.current_heading = euler[2]


    def frontRangeCB(self,range):
        self.front_range = range.ranges[0]

    def rightRangeCB(self,range):
        self.right_range = range.ranges[0]
        
    def leftRangeCB(self,range):
        self.left_range = range.ranges[0]


            # Transition state and restart the timer
    def transition(self, newState):
        state = newState
        self.state_start_time = time.time()
        return state

    def init(self):
        rospy.init_node('wall_follower_multirange', anonymous=True)
        rospy.sleep(3.)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.poseCB)
        rospy.Subscriber("/raw_imu", Imu, self.imuCB)
        rospy.Subscriber("/multi_ranger/front_range_sensor_value", LaserScan, self.frontRangeCB)
        rospy.Subscriber("/multi_ranger/right_range_sensor_value", LaserScan, self.rightRangeCB)
        rospy.Subscriber("/multi_ranger/left_range_sensor_value", LaserScan, self.leftRangeCB)

        rospy.sleep(1.)

        rospy.wait_for_service('enable_motors')
        enable_motors  = rospy.ServiceProxy('enable_motors', EnableMotors)
        enable_motors(True)

    def rosloop(self):
        bug_controller = ComAngleLoopController()

        bug_controller.init(1.0,0.5)
        twist = Twist()
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.state = "TAKE_OFF"
        rate = rospy.Rate(10) # 10hz

        prev_heading = 0.0



        while not rospy.is_shutdown():



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
                if self.distance_to_goal<0.5 and self.coord_index is len(self.goal_coord_x)-1:
                    self.state = self.transition("STOP")
            if self.state =="TURN_TO_GOAL":
                if time.time()-self.state_start_time > 1 and logicIsCloseTo(self.current_heading,wraptopi(self.angle_to_goal),0.1):
                    self.state = self.transition("STATE_MACHINE")
                    
                    
            if self.state == "TAKE_OFF":
                twist.linear.z = 0.1;
            if self.state =="STATE_MACHINE":
                twist = bug_controller.stateMachine(self.front_range,self.right_range, self.left_range, self.current_heading, wraptopi(self.angle_to_goal), self.distance_to_goal)
            if self.state =="TURN_TO_GOAL":
                twist.linear.x = 0.0;
                twist.linear.y = 0.0;
                twist.linear.z = 0.0;
                twist.angular.z = 0.3;
            if self.state =="STOP":
                twist.linear.x = 0.0;
                twist.linear.y = 0.0;
                twist.angular.z = 0.0;
            enable_motors  = rospy.ServiceProxy('enable_motors', EnableMotors)
            enable_motors(True)
            print self.state
            pub.publish(twist)
            rate.sleep()



if __name__ == '__main__':

    bug_gazebo = bug_gazebo()
    bug_gazebo.init()
    try:
        bug_gazebo.rosloop()
    except rospy.ROSInterruptException:
        pass
