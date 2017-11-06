// This program publishes randomly-generated velocity messages for a robot.
#include <ros/ros.h>

// For geometry_msgs::Twist 
#include <geometry_msgs/Twist.h>

// For sensor_msgs::LaserScan 
#include <sensor_msgs/LaserScan.h>

// For rand() and RAND_MAX
#include <stdlib.h>

#include <math.h>
#include <string.h>

using namespace std;

class FollowWall {

public:
	FollowWall(string name) {

		this->name = name;
		this->signal = (rand() % 2) * 2 - 1;;
		this->counter = 0;
		this->next = (rand() % 10 + 1) * 10;

		// Initiate the publisher.
		pub = nh.advertise<geometry_msgs::Twist>(name + "/cmd_vel", 1000);

		// Initiate the subscribers.
		// We subscribe to the robots lasers to detect walls.
		sub0 = nh.subscribe(name + "/laser0", 1000, &FollowWall::laser0Handler, this);
		sub1 = nh.subscribe(name + "/laser1", 1000, &FollowWall::laser1Handler, this);

	}

private:
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Subscriber sub0;
	ros::Subscriber sub1;
	sensor_msgs::LaserScan msg0;
	sensor_msgs::LaserScan msg1;
	int signal; // Determines whether the robot rotates counterclockwise or clockwise.
	int counter; // Incremented in each robot movement published message.
	int next; // When to reset the counter and randomize the signal.
	string name; // Name of the robot to move.

	void laser0Handler(const sensor_msgs::LaserScan& msg) {
		this->msg0 = msg;
		ROS_INFO_STREAM("LASER 0: " << this->msg0);
		move();
	}

	void laser1Handler(const sensor_msgs::LaserScan& msg) {
		this->msg1 = msg;
		ROS_INFO_STREAM("LASER 0: " << this->msg1);
		move();
	}

	void move() {

		this->counter++;	

		// Whenever the counter reaches next, reset it, randomize the signal 
		// and randomize the next check.
		if (this->counter == this->next) {
			this->counter = 0;
			this->signal = (rand() % 2) * 2 - 1;
			this->next = (rand() % 10 + 1) * 10;
		}

		// Create and fill in the message. 
		// The other four fields, which are ignore by the robot, default to 0.
		geometry_msgs::Twist out_msg;
		out_msg.linear.x = double(rand())/double(RAND_MAX);
		out_msg.angular.z = this->signal * double(rand())/double(RAND_MAX);

		// Publish the message.
		this->pub.publish(out_msg);

		// Send a message to rosout with the details.
		ROS_INFO_STREAM("Sending random velocity command to " << this->name << ":" 
			<< " linear=" << out_msg.linear.x << " angular=" << out_msg.angular.z);
	}

};

int main(int argc, char **argv) {

	// Seed the random number generator.
	srand((unsigned) time(NULL) * getpid());

	// Initialize the ROS system and become a node.
	ros::init(argc, argv, "publish_velocity");

	// Create an object of the FollowWall class that will take care of everything.
	string name(argv[1]);
	FollowWall *followWall = new FollowWall(name);

	// Let ROS take over.
	ros::spin();

}