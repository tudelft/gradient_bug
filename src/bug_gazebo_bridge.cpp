/*
 * bug_gazebo_onboard_prep.cpp
 *
 *  Created on: Aug 7, 2018
 *      Author: knmcguire
 */



#include "ros/ros.h"
#include "ros/time.h"
#include <tf/transform_datatypes.h>

#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Range.h"
#include "hector_uav_msgs/EnableMotors.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"


#include <sstream>

extern "C" {

#include "../lib/wallfollowing_multiranger_onboard.h"
}

// laser range callback
float front_range;
float right_range;
float left_range;
float height;
float heading;

void frontRangeCB(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	front_range = msg->ranges[0];
}

void rightRangeCB(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	right_range = msg->ranges[0];
}

void leftRangeCB(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	left_range = msg->ranges[0];
}

void sonarHeightCB(const sensor_msgs::Range::ConstPtr& msg)
{
	height = msg->range;
}

void poseCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

	tf::Matrix3x3 mat(q);
	double yaw, pitch, roll;

	mat.getEulerYPR(yaw, pitch, roll);
	heading = yaw;

}

int main(int argc, char **argv)
{
	// Init functions
	ros::init(argc, argv, "bug_gazebo_onboard_prep");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);



	// Subscribe to gazebo messages
	ros::Subscriber sub_frontrange = n.subscribe("/multi_ranger/front_range_sensor_value", 1000, frontRangeCB);
	ros::Subscriber sub_rightrange = n.subscribe("/multi_ranger/right_range_sensor_value", 1000, rightRangeCB);
	ros::Subscriber sub_leftrange = n.subscribe("/multi_ranger/left_range_sensor_value", 1000, leftRangeCB);
	ros::Subscriber sub_sonarheight = n.subscribe("/sonar_height", 1000, sonarHeightCB);
	ros::Subscriber sub_pose = n.subscribe("/ground_truth_to_tf/pose", 1000, poseCB);


	// Publish to gazebo controller
	ros::Publisher pub_cmdvel = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	//sensor_msgs::LaserScan::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>("/multi_ranger/front_range_sensor_value");

	ros::Duration(2).sleep();
	ros::service::waitForService("enable_motors", -1);
	ros::ServiceClient client = n.serviceClient<hector_uav_msgs::EnableMotors>("enable_motors");
	hector_uav_msgs::EnableMotors srv;
	srv.request.enable = true;
	client.call(srv);
	// Init wall follower
	wall_follower_init(1.0, 0.5);

	bool taken_off = false;
	geometry_msgs::Twist twist_msg;


	while (ros::ok())
	{

		if(taken_off == false )
		{
			twist_msg.linear.x = 0;
			twist_msg.linear.y = 0;
			twist_msg.linear.z = 0.5;
			if(height > 1.0)
			{
				taken_off = true;
			}
		}else{
			//testRange(front_range, right_range, left_range);
			float vel_x;
			float vel_y;
			float vel_w;

			wall_follower(&vel_x, &vel_y, &vel_w, front_range,  right_range, heading,  1);

			twist_msg.linear.x = vel_x;
			twist_msg.linear.y = vel_y;
			twist_msg.angular.z = vel_w;
			twist_msg.linear.z = 0.0;
		}


		pub_cmdvel.publish(twist_msg);

		// Spine once and sleep to keep loop at same rate
		ros::spinOnce();
		loop_rate.sleep();

	}

	return 0;
}
