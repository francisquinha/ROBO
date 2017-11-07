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

const float ERROR = 0.05;
const float BOTTOM = 0.95;
const float TOP = 2.3;

class Wall {

public:
	Wall(string name) {

		this->name = name;
		this->signal = (rand() % 2) * 2 - 1;;
		this->counter = 0;
		this->next = (rand() % 10 + 1) * 10;

		this->laser0_bot = FLT_MAX;
		this->laser1_bot = FLT_MAX;
		this->laser0_top = FLT_MAX;
		this->laser1_top = FLT_MAX;

		this->state = unknown;

		// Initiate the publisher.
		pub = nh.advertise<geometry_msgs::Twist>(name + "/cmd_vel", 1000);

		// Initiate the subscribers.
		// We subscribe to the robots lasers to detect walls.
		sub0 = nh.subscribe(name + "/laser0", 1000, &Wall::laser0Handler, this);
		sub1 = nh.subscribe(name + "/laser1", 1000, &Wall::laser1Handler, this);

/*		ros::Rate rate(2);
		while(ros::ok()) {
			move();
			rate.sleep();
		}*/
	}

private:
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Subscriber sub0;
	ros::Subscriber sub1;
	int signal; // Determines whether the robot rotates counterclockwise or clockwise.
	int counter; // Incremented in each robot movement published message.
	int next; // When to reset the counter and randomize the signal.
	string name; // Name of the robot to move.
	float laser0_bot; // Left side laser, bottom value within range
	float laser1_bot; // Right side laser, bottom value within range
	float laser0_top; // Left side laser, top value within range
	float laser1_top; // Right side laser, top value within range
	enum State {unknown, forward, turn_right};
	State state;

	void laser0Handler(const sensor_msgs::LaserScan& msg) {
		laserParser(msg, this->laser0_bot, this->laser0_top);
		ROS_INFO_STREAM("Laser 0 bottom: " << this->laser0_bot << ". Laser 0 top:" << this->laser0_top);
		move();
	}

	void laser1Handler(const sensor_msgs::LaserScan& msg) {
		laserParser(msg, this->laser1_bot, this->laser1_top);
		ROS_INFO_STREAM("Laser 1 bottom: " << this->laser1_bot << ". Laser 1 top:" << this->laser1_top);
		move();
	}

	void laserParser(const sensor_msgs::LaserScan& msg, float& bot_value, float& top_value) {
		bot_value = msg.ranges[0];
		top_value = msg.ranges[msg.ranges.size() - 1];
		if (bot_value <= msg.range_min || bot_value >= msg.range_max)
			bot_value = FLT_MAX;
		if (top_value <= msg.range_min || top_value >= msg.range_max)
			top_value = FLT_MAX;
	}

	void move() {
		if (this->laser0_top == FLT_MAX && this->laser1_top == FLT_MAX && this->laser0_bot == FLT_MAX 
			&& this->laser1_bot == FLT_MAX && this->state == unknown) 
			randomMove();
		else
			wallMove();
	}

	void randomMove() {

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

	void wallMove() {
		// Create the message. 
		geometry_msgs::Twist out_msg;
		if (this->laser1_bot >= BOTTOM - ERROR && this->laser1_bot <= BOTTOM + ERROR 
			&& this->laser1_top >= TOP - ERROR && this->laser1_top <= TOP + ERROR) {
			// Move forward
			this->state = forward;
			out_msg.linear.x = 1;
			out_msg.angular.z = 0;
			this->pub.publish(out_msg);
			ROS_INFO_STREAM("Sending forward velocity command to " << this->name << ":" 
			<< " linear=" << out_msg.linear.x << " angular=" << out_msg.angular.z);
		}
		else if (this->laser1_bot >= BOTTOM - ERROR && this->laser1_bot <= BOTTOM + ERROR 
			&& this->laser1_top == FLT_MAX) {
			// Will have to turn right
			out_msg.linear.x = 1;
			out_msg.angular.z = 0;
			this->state = turn_right;
			this->pub.publish(out_msg);
			ROS_INFO_STREAM("Sending forward velocity command to " << this->name << ":" 
			<< " linear=" << out_msg.linear.x << " angular=" << out_msg.angular.z);
		}
		else if (this->laser1_bot == FLT_MAX && this->laser1_top == FLT_MAX && this->state == turn_right) {
			// Turn hard right
			out_msg.linear.x = 1;
			out_msg.angular.z = - M_PI / 2;
			this->pub.publish(out_msg);
			ROS_INFO_STREAM("Sending turn right velocity command to " << this->name << ":" 
			<< " linear=" << out_msg.linear.x << " angular=" << out_msg.angular.z);
		}
		else randomMove();
	}

};

int main(int argc, char **argv) {
	
	// Seed the random number generator.
	srand((unsigned) time(NULL) * getpid());

	// Initialize the ROS system and become a node.
	ros::init(argc, argv, "publish_velocity");

	// Create an object of the Wall class that will take care of everything.
	string name(argv[1]);
	Wall *wall = new Wall(name);

	// Let ROS take over.
	ros::spin();

}