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




class WF_gazebo:
    # Callbacks
    front_range = 0.0
    right_range = 0.0
    altitude = 0.0
    state = "TAKE_OFF"
    current_heading = 0.0

    def poseCB(self,state):
        self.altitude=state.pose[1].position.z


    def imuCB(self,imu):
        explicit_quat = [imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w]

        euler = tf.transformations.euler_from_quaternion(explicit_quat)
        self.current_heading = euler[2]


    def frontRangeCB(self,range):
        self.front_range = range.ranges[0]

    def rightRangeCB(self,range):
        self.right_range = range.ranges[0]


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

        rospy.wait_for_service('enable_motors')
        enable_motors  = rospy.ServiceProxy('enable_motors', EnableMotors)
        enable_motors(True)

    def rosloop(self):
        wall_follower = WallFollower()
        wall_follower.init(1.0,1.0)
        twist = Twist()
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)


        rate = rospy.Rate(10) # 10hz
        while not rospy.is_shutdown():
            if self.state == "TAKE_OFF":
                twist.linear.z = 0.1;
                if self.altitude > 0.5:
                    self.state = self.transition("WALL_FOLLOWING")
            if self.state =="WALL_FOLLOWING":
                twist = wall_follower.wall_follower(self.front_range,self.right_range, self.current_heading)

            pub.publish(twist)
            rate.sleep()



if __name__ == '__main__':

    wf_gazebo = WF_gazebo()
    wf_gazebo.init()
    try:
        wf_gazebo.rosloop()
    except rospy.ROSInterruptException:
        pass
