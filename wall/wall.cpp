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

class Wall {

public:
	Wall(string name) {

		this->name = name;
		this->signal = (rand() % 2) * 2 - 1;;
		this->counter = 0;
		this->next = (rand() % 10 + 1) * 10;

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
	float laser0_min; // Left side laser, min value within range
	float laser1_min; // Right side laser, min value within range
	float laser0_max; // Left side laser, max value within range
	float laser1_max; // Right side laser, max value within range

	void laser0Handler(const sensor_msgs::LaserScan& msg) {
		laserParser(msg, this->laser0_min, this->laser0_max);
		ROS_INFO_STREAM("Laser 0 min: " << this->laser0_min << ". Laser 0 max:" << this->laser0_max);
		move();
	}

	void laser1Handler(const sensor_msgs::LaserScan& msg) {
		laserParser(msg, this->laser1_min, this->laser1_max);
		ROS_INFO_STREAM("Laser 1 min: " << this->laser1_min << ". Laser 1 max:" << this->laser1_max);
		move();
	}

	void laserParser(const sensor_msgs::LaserScan& msg, float& min_value, float& max_value) {
		min_value = FLT_MAX;
		max_value = 0;
		float new_value;
		for (int i = 0; i < msg.ranges.size(); i++) {
			new_value = msg.ranges[i];
			if (new_value >= msg.range_min && new_value <= msg.range_max) {
				if (new_value < min_value) 
					min_value = new_value;
				if (new_value > max_value)
					max_value = new_value;
			}
		}
		if (min_value == FLT_MAX)
			min_value = 0;
	}

	void move() {
		if (this->laser0_max == 0 && this->laser1_max == 0)
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
		randomMove();
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