# include "../include/wall_following.h"

using namespace std;

namespace robo_stdr_reactive
{
	WallFollowing::WallFollowing(int argc, char **argv)
	{
		if(argc != 4)
		{
			ROS_ERROR(
			"Usage : stdr_line following <robot_frame_id> <laser0_frame_id> <laser1_frame_id>");
			exit(0);
		}

		laser0_topic = string("/") + string(argv[1]) + string("/") + string(argv[2]);
		laser1_topic = string("/") + string(argv[1]) + string("/") + string(argv[3]);
		speeds_topic = string("/") + string(argv[1]) + string("/cmd_vel");

		subscriber0 = n.subscribe(laser0_topic.c_str(), 1, &WallFollowing::callback, this);
		subscriber1 = n.subscribe(laser1_topic.c_str(), 1, &WallFollowing::callback, this);

		cmd_vel_pub = n.advertise<geometry_msgs::Twist>(speeds_topic.c_str(), 1);
	}


	WallFollowing::~WallFollowing(void){}

  	void WallFollowing::callback(const sensor_msgs::LaserScan& msg)
	{
	    scan = msg;
    	geometry_msgs::Twist cmd;
		
		int laserSide = -1;

		if(scan.header.frame_id.compare(laser0_topic)){ //left side;
			
			laserSide = LEFT_SIDE;

		}else if(scan.header.frame_id.compare(laser0_topic)){ //right side
		
			laserSide = RIGHT_SIDE;
		}

		cout << "scan reading  " << scan.header.frame_id << endl;
    	
    
    	
    	//read the info from the laser sensor
    	for(int i = 0; i < scan.ranges.size(); i++)
    	{
			float distance = scan.ranges[i];
			float sensor_angle = -100.0 + rad2deg(scan.angle_increment) * i;
			
		}
    	
		cmd_vel_pub.publish(cmd);
	}

	/**
     * Converts angle in radians to degrees
	 */
	float WallFollowing::rad2deg(float radian)
	{
		return 	radian * (180 / 3.1415);
	}

	/**
     * Converts angle in degrees to radians
	 */
	float WallFollowing::deg2rad(float degree)
	{
		return degree * (3.1415 / 180);
	}
}






