/*
 * bug_gazebo_onboard_prep.cpp
 *
 *  Created on: Aug 7, 2018
 *      Author: knmcguire
 */



#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"

#include <sstream>

		extern "C" {

#include "wallfollowing_multiranger_onboard.h"
		}

// laser range callback
float front_range;
float right_range;
float left_range;

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


	while (ros::ok())
	{
		testRange(front_range, right_range, left_range);

	    // Spine once and sleep to keep loop at same rate
	    ros::spinOnce();
	    loop_rate.sleep();

	}

	return 0;
}
