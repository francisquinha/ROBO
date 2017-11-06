#ifndef STDR_LINE_FOLLOWING
#define STDR_LINE_FOLLOWING

#include <iostream>
#include <cstdlib>
#include <cmath>
#include <ctime>

#include <ros/package.h>
#include "ros/ros.h"

#include <stdr_msgs/RobotIndexedVectorMsg.h>

#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>

namespace robo_stdr_reactive
{

	const int LEFT_SIDE = 0;
	const int RIGHT_SIDE = 1;

	class WallFollowing
	{
	private:
		sensor_msgs::LaserScan scan;
		ros::Subscriber subscriber0;
		ros::Subscriber subscriber1;
		ros::NodeHandle n;
		std::string laser0_topic; //left
		std::string laser1_topic; //right
		std::string speeds_topic;
		ros::Publisher cmd_vel_pub;
		
		float rad2deg(float radian);
		float deg2rad(float degree);
		
	public:
		WallFollowing(int argc,char **argv);
		~WallFollowing(void);
		void callback(const sensor_msgs::LaserScan& msg);
  	};
}

#endif
