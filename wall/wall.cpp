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

const float MIN_ERROR = 0.01;
const float MAX_ERROR = 0.1;
const float DISTANCE = 1;

class Wall {

public:
	Wall(string name) {

		this->name = name;
		this->signal = (rand() % 2) * 2 - 1;;
		this->counter = 0;
		this->next = (rand() % 10 + 1) * 10;

		this->left = FLT_MAX;
		this->right = FLT_MAX;

		this->state = random;
		this->laser = unknown;

		// Initiate the publisher.
		pub = nh.advertise<geometry_msgs::Twist>(name + "/cmd_vel", 1000);

		// Initiate the subscribers.
		// We subscribe to the robots lasers to detect walls.
		sub0 = nh.subscribe(name + "/left", 1000, &Wall::leftHandler, this);
		sub1 = nh.subscribe(name + "/right", 1000, &Wall::rightHandler, this);

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
	float left; // Left side laser, min value within range
	float right; // Right side laser, min value within range
	enum State {random, wall};
	State state;
	enum Laser {unknown, left_laser, right_laser};
	Laser laser;

	void leftHandler(const sensor_msgs::LaserScan& msg) {
		laserParser(msg, this->left);
		ROS_INFO_STREAM("Left laser: " << this->left);
		move();
	}

	void rightHandler(const sensor_msgs::LaserScan& msg) {
		laserParser(msg, this->right);
		ROS_INFO_STREAM("Right laser: " << this->right );
		move();
	}

	void laserParser(const sensor_msgs::LaserScan& msg, float& value) {
		value = FLT_MAX;
		float new_value;
		for (int i = 0; i < msg.ranges.size(); i++) {
			new_value = msg.ranges[i];
			if (new_value >= msg.range_min && new_value <= msg.range_max && new_value < value)
				value = new_value;
		}
	}

	void move() {
		if (this->left == FLT_MAX && this->right == FLT_MAX && this->state == random) 
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
		ROS_INFO_STREAM("Random velocity: "	<< " linear=" << out_msg.linear.x << " angular=" << out_msg.angular.z);
	}

	void wallMove() {
		this->state = wall;
		// Create the message. 
		geometry_msgs::Twist out_msg;
		float laser_min;
		int signal;
		if (this->left == FLT_MAX && this->right < FLT_MAX) {
			laser_min = this->right;
			this->laser = right_laser;
			ROS_INFO_STREAM("RIGHT");
			signal = 1;
		}
		else if (this->right == FLT_MAX && this->left < FLT_MAX) {
			laser_min = this->left;
			this->laser = left_laser;
			ROS_INFO_STREAM("LEFT");
			signal = -1;
		}
		else if (this->laser == right_laser) {
			laser_min = this->right;
			signal = 1;						
		}
		else if (this->laser == left_laser) {
			laser_min = this->left;
			signal = -1;						
		}
		else if (this->right < this->left) {
			laser_min = this->right;
			this->laser = right_laser;
			ROS_INFO_STREAM("RIGHT");
			signal = 1;
		}
		else {
			laser_min = this->left;
			this->laser = left_laser;
			ROS_INFO_STREAM("LEFT");
			signal = -1;			
		}
		if (laser_min >= DISTANCE - MIN_ERROR && laser_min <= DISTANCE + MIN_ERROR) {
			// Move forward
			out_msg.linear.x = 1;
			out_msg.angular.z = 0;
			ROS_INFO_STREAM("FORWARD");
		}
		else if (laser_min < DISTANCE - MAX_ERROR) {
			// Turn hard left if right, hard right if left
			out_msg.linear.x = 0.1;
			out_msg.angular.z = signal * M_PI / 4;
			ROS_INFO_STREAM("HARDLEFT1 HARDRIGHT0");
		}
		else if (laser_min < DISTANCE - MIN_ERROR) {
			// Turn left if right, right if left
			out_msg.linear.x = 1;
			out_msg.angular.z = signal * M_PI / 8;
			ROS_INFO_STREAM("LEFT1 RIGHT0");
		}
		else if (laser_min > DISTANCE + MAX_ERROR) {
			// Turn hard right if right, hard left if left
			out_msg.linear.x = 0.5;
			out_msg.angular.z = - signal * M_PI / 4;
			ROS_INFO_STREAM("HARDRIGHT1 HARDLEFT0");
		}
		else if (laser_min > DISTANCE + MIN_ERROR) {
			// Turn right if right, left if left
			out_msg.linear.x = 1;
			out_msg.angular.z = - signal * M_PI / 8;
			ROS_INFO_STREAM("RIGHT1 LEFT0");
		}
		else {
			// Do nothing and go back to random
			out_msg.linear.x = 0;
			out_msg.angular.z = 0;
			this->state = random;
			this->laser = unknown;
		}
		this->pub.publish(out_msg);
		ROS_INFO_STREAM("Velocity: "	<< " linear=" << out_msg.linear.x << " angular=" << out_msg.angular.z);
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