#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from hector_uav_msgs.srv import EnableMotors
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Imu
import time
import tf
import math
from _ast import IsNot
from amcl.cfg.AMCLConfig import inf




class WallFollower:

    ref_distance_from_wall = 1.0
    max_speed = 0.2
    front_range = 0.0
    right_range = 0.0
    max_rate = 0.5
    state_start_time = 0
    state = "FORWARD"
    previous_heading = 0.0;
    angle=2000
    calculate_angle_first_time = True;
    around_corner_first_turn = True;

    def init(self,new_ref_distance_from_wall,max_speed_ref = 0.2):
        self.ref_distance_from_wall = new_ref_distance_from_wall
        self.state = "TURN_TO_FIND_WALL"
        self.max_speed = max_speed_ref
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

    def twistForwardAlongWall(self, range):
        twist = Twist()
        twist.linear.x = self.max_speed
        if  self.logicIsCloseTo(self.ref_distance_from_wall, range, 0.1) == False:
            if range>self.ref_distance_from_wall:
                twist.linear.y =  - self.max_speed/3
            else:
                twist.linear.y =  self.max_speed /3

        return twist

    def twistTurn(self,rate):
        v = 0.0
        w = rate
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist

    def twistTurnandAdjust(self,rate, range):
        v = 0.0
        w = rate
        twist = Twist()

        if  self.logicIsCloseTo(self.ref_distance_from_wall, range, 0.1) == False:
            if range>self.ref_distance_from_wall:
                twist.linear.y =  - self.max_speed/3
            else:
                twist.linear.y =  self.max_speed /3
        twist.linear.x = v
        twist.angular.z = w
        return twist

    def twistTurnAroundCorner(self, radius):
        v = self.max_speed
        w = -v/radius
        twist = Twist()
        twist.linear.x = v
        twist.angular.z = w
        return twist




    def logicIsCloseTo(self,real_value = 0.0, checked_value =0.0, margin=0.05):

        if real_value> checked_value-margin and real_value< checked_value+margin:
            return True
        else:
            return False

    # Transition state and restart the timer
    def transition(self, newState):
        state = newState
        self.state_start_time = time.time()
        return state

    def wall_follower(self, front_range, right_range, current_heading):



        #handle state transitions
        if self.state == "TAKE_OFF":
            if self.altitude > 0.5:
                self.state = self.transition("FORWARD")
        elif self.state == "FORWARD":
            if front_range < self.ref_distance_from_wall:
                self.state = self.transition("TURN_TO_FIND_WALL")
        elif self.state == "HOVER":
            print state
        elif self.state == "TURN_TO_FIND_WALL":
            print(front_range,right_range)
            if (right_range < self.ref_distance_from_wall+0.4 and front_range < self.ref_distance_from_wall+0.4):
                self.previous_heading = current_heading;
                self.angle = 1.57 - math.atan(front_range/right_range)
                self.state = self.transition("TURN_TO_ALLIGN_TO_WALL")
            if (right_range < 1.0 and front_range > 2.0):
                self.around_corner_first_turn = True
                self.state = self.transition("ROTATE_AROUND_WALL")
        elif self.state =="TURN_TO_ALLIGN_TO_WALL":
            print current_heading
            print self.previous_heading
            print self.angle
            if current_heading-self.previous_heading>self.angle - 0.05:
                self.state = self.transition("FORWARD_ALONG_WALL")

        elif self.state =="FORWARD_ALONG_WALL":
            if right_range > 2:
                self.around_corner_first_turn = True
                self.state = self.transition("ROTATE_AROUND_WALL")
            if front_range < self.ref_distance_from_wall:
                self.state = self.transition("ROTATE_IN_CORNER")
                self.previous_heading = current_heading;
        elif self.state =="ROTATE_AROUND_WALL":
            if front_range < self.ref_distance_from_wall+0.3:
                self.state = self.transition("TURN_TO_FIND_WALL")
        elif self.state == "ROTATE_IN_CORNER":
            if current_heading-self.previous_heading > 0.8 - 0.1:
                self.state = self.transition("TURN_TO_FIND_WALL")


        print self.state



        #handle state ations
        if self.state == "TAKE_OFF":
            twist = self.take_off()
        elif self.state == "FORWARD":
            twist = self.twistForward()
        elif self.state == "HOVER":
            twist = self.hover()
        elif self.state == "TURN_TO_FIND_WALL":
            twist = self.hover()
            if (time.time() - self.state_start_time) > 1:
                twist = self.twistTurn(self.max_rate);
        elif self.state =="TURN_TO_ALLIGN_TO_WALL":
            twist = self.hover()
            if (time.time() - self.state_start_time) > 1:
                twist = self.twistTurn(self.max_rate)
        elif self.state =="FORWARD_ALONG_WALL":
            twist = self.twistForwardAlongWall(right_range)
        elif self.state == "ROTATE_AROUND_WALL":
            print right_range
            if right_range>self.ref_distance_from_wall+0.5 and self.around_corner_first_turn:
                twist = self.twistTurn(-self.max_rate)
            elif right_range>self.ref_distance_from_wall+0.5:
                twist = self.twistTurnandAdjust(self.max_rate,right_range)
            else:
                twist = self.twistTurnAroundCorner(self.ref_distance_from_wall)
                self.around_corner_first_turn = False

        elif self.state == "ROTATE_IN_CORNER":
            twist = self.twistTurn(self.max_rate);


        return twist








if __name__ == '__main__':
    try:
        wall_follower()
    except rospy.ROSInterruptException:
        pass
