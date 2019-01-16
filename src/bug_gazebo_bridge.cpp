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
#include "rosgraph_msgs/Clock.h"

#include <random>

#include <sstream>
#include <time.h>
#include <algorithm>
#include <string>

extern "C" {

#include "/home/knmcguire/Software/crazyflie/crazyflie-firmware/src/lib/wallfollowing_multiranger_onboard/wallfollowing_multiranger_onboard.h"
#include "/home/knmcguire/Software/crazyflie/crazyflie-firmware/src/lib/wallfollowing_multiranger_onboard/lobe_navigation.h"
#include "/home/knmcguire/Software/crazyflie/crazyflie-firmware/src/lib/wallfollowing_multiranger_onboard/com_bug_with_looping.h"
#include "/home/knmcguire/Software/crazyflie/crazyflie-firmware/src/lib/wallfollowing_multiranger_onboard/wallfollowing_with_avoid.h"
#include "/home/knmcguire/Software/crazyflie/crazyflie-firmware/src/lib/wallfollowing_multiranger_onboard/com_bug_with_looping_and_avoid.h"
#include "/home/knmcguire/Software/crazyflie/crazyflie-firmware/src/lib/wallfollowing_multiranger_onboard/lobe_bug_with_looping.h"
#include "/home/knmcguire/Software/crazyflie/crazyflie-firmware/src/lib/wallfollowing_multiranger_onboard/median_filter.h"
#include "/home/knmcguire/Software/crazyflie/crazyflie-firmware/src/lib/wallfollowing_multiranger_onboard/gradient_bug_with_looping.h"


}

//#define REVERSE

// laser range callback
float front_range;
float right_range;
float left_range;
float back_range;

float height;
float heading;
float pos_x;
float pos_y;
float other_pos_x;
float other_pos_y;


std::default_random_engine generator;

float findMinimum(std::vector<float> array, int size)
{
    float smallest = array[0] ;
    for ( int i=1;  i < size;  ++i )
        if ( array[i] < smallest )
             smallest = array[i] ;

    return smallest;
}
void frontRangeCB(const sensor_msgs::LaserScan::ConstPtr& msg)
{


	front_range =findMinimum(msg->ranges,3);
}

void backRangeCB(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	back_range = findMinimum(msg->ranges,3);
}

void rightRangeCB(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	right_range = findMinimum(msg->ranges,3);
}

void leftRangeCB(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	left_range = findMinimum(msg->ranges,3);
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

	pos_x = msg->pose.position.x;
	pos_y = msg->pose.position.y;

}

void otherPoseCB(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
	tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);


	other_pos_x = msg->pose.position.x;
	other_pos_y = msg->pose.position.y;

}
void noisy_twist(geometry_msgs::Twist *twist, double std_lin, double std_rate)
{
	const double mean_lin_x = twist->linear.x;
	const double mean_lin_y = twist->linear.y;
	const double mean_rate = twist->angular.z;

	const double stddev_lin = std_lin;
	const double stddev_rate = std_rate;

	std::normal_distribution<double> dist_lin_x(mean_lin_x, stddev_lin);
	std::normal_distribution<double> dist_lin_y(mean_lin_y, stddev_lin);
	std::normal_distribution<double> dist_rate(mean_rate, stddev_rate);


	twist->linear.x = dist_lin_x(generator);
	twist->linear.y = dist_lin_y(generator);
	twist->angular.z = dist_rate(generator);

}
static float wraptopi(float number)
{

	if(number>(float)M_PI)
		return (number-(float)(2*M_PI));
	else if(number< (float)(-1*M_PI))
		return (number+(float)(2*M_PI));
	else
		return (number);

}
int main(int argc, char **argv)
{
	// Init functions
	ros::init(argc, argv, "bug_gazebo_onboard_prep");
	ros::NodeHandle n;
	ros::Rate loop_rate(30);

	std::string ns = ros::this_node::getNamespace();

	struct MedianFilterInt medFiltRssibeacon;
	init_median_filter_i(&medFiltRssibeacon,101);

	// Subscribe to gazebo messages
	ros::Subscriber sub_backrange = n.subscribe("multi_ranger/back_range_sensor_value", 1000, backRangeCB);

	ros::Subscriber sub_frontrange = n.subscribe("multi_ranger/front_range_sensor_value", 1000, frontRangeCB);
	ros::Subscriber sub_rightrange = n.subscribe("multi_ranger/right_range_sensor_value", 1000, rightRangeCB);
	ros::Subscriber sub_leftrange = n.subscribe("multi_ranger/left_range_sensor_value", 1000, leftRangeCB);
	ros::Subscriber sub_sonarheight = n.subscribe("sonar_height", 1000, sonarHeightCB);
	ros::Subscriber sub_pose = n.subscribe("ground_truth_to_tf/pose", 1000, poseCB);
	//ros::Subscriber sub_clock = n.subscribe("/clock", 1000, clockCB);

	ros::Subscriber sub_other_pose;
	float begin_direction = 0;
	if(ns=="//UAV1"){
		begin_direction = 1;
		sub_other_pose=n.subscribe("/UAV2/ground_truth_to_tf/pose", 1000, otherPoseCB);
	}else if(ns=="//UAV2")
	{
		sub_other_pose=n.subscribe("/UAV1/ground_truth_to_tf/pose", 1000, otherPoseCB);
		begin_direction = -1;
	}

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
	//wall_follower_init(0.5, 0.5);
	//init_lobe_navigator();
	//init_com_bug_loop_controller(0.5, 0.5);
	//init_wall_follower_and_avoid_controller(0.5, 0.5,begin_direction);
	//init_com_bug_loop_avoid_controller(0.5, 0.5);
	//init_lobe_bug_loop_controller(0.5, 0.5);
	//if(ns=="//UAV1")
	std::string tmp;
	std::stringstream ss(ns);
	int own_id=(int)(ns.back()- '0');
	std::cout<<own_id<<std::endl;


	if (own_id == 1)
	init_gradient_bug_loop_controller(0.5, 0.5,2.35);
	else
		init_gradient_bug_loop_controller(0.5, 0.5,-0.8);

	bool taken_off = false;
	geometry_msgs::Twist twist_msg;

	//time_t start, finish;

	//#time(&start);

	float start_time = 0;

	ros::Duration((own_id-1)*10).sleep();

	while (ros::ok())
	{

		//Fix for now for the gazebo bug
		if(height > 2.99)
			height = 0;

		if(taken_off == false )
		{
			twist_msg.linear.x = 0;
			twist_msg.linear.y = 0;
			twist_msg.linear.z = 0.5;
			if(height > 1.0)
			{
				taken_off = true;
				start_time = ros::Time::now().toSec();
			}
		}else{
			//testRange(front_range, right_range, left_range);
			float vel_x=0;
			float vel_y=0;
			float vel_w=0;

			//wall_follower(&vel_x, &vel_y, &vel_w, front_range,  left_range, heading,  -1);
			float beacon_angle = atan2(pos_y,pos_x);
			float beacon_distance = sqrt(pos_x*pos_x+pos_y*pos_y);




			std::normal_distribution<float> dist_distance((float)(beacon_distance), 3);
			std::normal_distribution<float> dist_angle((float)(beacon_angle), 0.01);

			float noisy_beacon_distance =dist_distance(generator);
			float noisy_beacon_angle = dist_angle(generator);

			//RSSI = Pn - 10*gamma*log10(distance)
			float Pn = -45.0f;
			float gamma_rssi = 4.0f;

			float noisy_RSSI = Pn - 10*gamma_rssi*log10(noisy_beacon_distance);
			float noisy_RSSI_bearing = noisy_RSSI+ (noisy_RSSI-Pn)*fabs(wraptopi(noisy_beacon_angle-heading));

			if(noisy_RSSI_bearing>44)
				noisy_RSSI_bearing = -44;

			uint8_t noisy_RSSI_bearing_uint8 = (uint8_t)(-1*noisy_RSSI);
			uint8_t rssi_beacon_filtered = (uint8_t)update_median_filter_i(&medFiltRssibeacon,noisy_RSSI_bearing_uint8);

			//printf("rssi %d\n",rssi_beacon_filtered);
			// std::normal_distribution<float> dist_rssi((float)(dummy_rssi), 10);

			//printf("heading %f beacon_angle %f noisybeaconangle %f pos_x %f pos_y %f\n",heading,beacon_angle,noisy_beacon_angle,pos_x,pos_y);
			/*			uint8_t dummy_rssi = 44+(uint8_t)(fabs(wraptopi(beacon_angle-heading))*20.0f);

		    std::normal_distribution<float> dist_rssi((float)(dummy_rssi), 10);
		    uint8_t noisy_rssi = dist_rssi(generator);*/
			float rssi_angle_save = 0;
			//lobe_navigator(&vel_x, &vel_y, &vel_w, &rssi_angle_save,front_range, left_range, heading, pos_x, pos_y,noisy_RSSI_bearing_uint8);
			//lobe_bug_loop_controller(&vel_x, &vel_y, &vel_w, front_range, left_range, right_range, heading, pos_x, pos_y,rssi_beacon_filtered);
			int state_wf;


			//std::cout<<" rssi_angle"<<rssi_angle_save<<std::endl;
			// COMBUG_LOOPING
			//com_bug_loop_controller(&vel_x, &vel_y, &vel_w, front_range, left_range, right_range, heading, pos_x, pos_y);

			// WALL FOLLOWING AND AVOID
			float diff_pos_y = pos_y-other_pos_y;
			float diff_pos_x= pos_x-other_pos_x;
			float other_drone_distance = sqrt(diff_pos_x*diff_pos_x + diff_pos_y*diff_pos_y);

			uint8_t rssi_other_drone =60;

			if ( other_drone_distance<2.0)
			{
				rssi_other_drone = 44;

			}else if( other_drone_distance<5.0) {

				rssi_other_drone = 50;

			}
			//std::cout<<other_drone_distance<<std::endl;

			bool id_priority = false;

			//std::cout<<ns<<std::endl;
			if(ns=="//UAV1")id_priority = true;

			bool outbound = true;
			double now_time = ros::Time::now().toSec();
			if(now_time-start_time>120)
				outbound = false;
			//std::cout<<now_time-start_time<<std::endl;

			if(front_range>0.2f||left_range>0.2f||back_range>0.2f||right_range>0.2f)
			{gradient_bug_loop_controller(&vel_x, &vel_y, &vel_w,&rssi_angle_save, &state_wf, front_range, left_range, right_range,
					back_range, heading, pos_x, pos_y,rssi_beacon_filtered, rssi_other_drone,0.8,id_priority,outbound);
			}else
			{
				if(front_range<=0.2f){
					vel_x += -0.2;
				}
				if(back_range<=0.2f){
					vel_x += 0.2;
				}
				if(left_range<=0.2f){
					vel_x += -0.2;
				}
				if(right_range<=0.2f){
					vel_x += 0.2;
				}

			}

			/*
#ifndef REVERSE
			wall_follower_and_avoid_controller(&vel_x, &vel_y, &vel_w, front_range, left_range, right_range, heading,pos_x, pos_y, rssi_other_drone);
#else
			printf("right_range %f, left_range %f\n",right_range, left_range);
			wall_follower_and_avoid_controller(&vel_x, &vel_y, &vel_w, back_range, left_range, right_range,-1* wraptopi(heading+3.14),pos_x, pos_y, rssi_other_drone);
			vel_x = -1*vel_x;
			vel_y = 1*vel_y;
			vel_w = -1*vel_w;

#endif
			 */

			//com_bug_loop_avoid_controller(&vel_x, &vel_y, &vel_w, front_range, left_range, right_range, heading, pos_x, pos_y,rssi_other_drone);
			if(now_time-start_time<420)
			{
				if (beacon_distance<2.0f && outbound == false)

				{
					twist_msg.linear.x = 0;
					twist_msg.linear.y = 0;
					twist_msg.angular.z = 0;
					twist_msg.linear.z = 0.0;
					std::cout<<"MADE IT"<<std::endl;
				}else{
					twist_msg.linear.x = vel_x;
					twist_msg.linear.y = vel_y;
					twist_msg.angular.z = vel_w;
					twist_msg.linear.z = 0.0;
				}
			}else{
				twist_msg.linear.x = 0;
				twist_msg.linear.y = 0;
				twist_msg.angular.z = 0;
				twist_msg.linear.z = 0.0;
				std::cout<<"BATTERY_LOW!"<<std::endl;
			}

		}

		noisy_twist(&twist_msg,0.05,0.1);


		pub_cmdvel.publish(twist_msg);

		// Spine once and sleep to keep loop at same rate
		ros::spinOnce();
		loop_rate.sleep();

	}

	return 0;
}
