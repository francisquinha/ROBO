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

class Wall {

public:
	Wall(string name, float distance) {

		this->name = name;
		this->distance = distance;
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
	float distance; // Distance to keep from wall.
	float left; // Left side laser, min value within range
	float right; // Right side laser, min value within range
	enum State {random, wall};
	State state;
	enum Laser {unknown, left_laser, right_laser};
	Laser laser;

	// Left laser handler: saves the left laser minimum value to this->left
	void leftHandler(const sensor_msgs::LaserScan& msg) {
		laserParser(msg, this->left);
		ROS_INFO_STREAM("Left laser: " << this->left);
		move();
	}

	// Right laser handler: saves the right laser minimum value to this->right
	void rightHandler(const sensor_msgs::LaserScan& msg) {
		laserParser(msg, this->right);
		ROS_INFO_STREAM("Right laser: " << this->right );
		move();
	}

	// Laser parser: finds and saves the laser minimum value within the range
	void laserParser(const sensor_msgs::LaserScan& msg, float& value) {
		value = FLT_MAX;
		float new_value;
		for (int i = 0; i < msg.ranges.size(); i++) {
			new_value = msg.ranges[i];
			if (new_value >= msg.range_min && new_value <= msg.range_max && new_value < value)
				value = new_value;
		}
	}

	// If none of the lasers sees anything tells the robot to move randomly, else tells it to follow the wall
	void move() {

		// See nothing and haven't seen anything before, move randomly
		if (this->left == FLT_MAX && this->right == FLT_MAX && this->state == random) 
			randomMove();

		// See wall or saw it recently, follow it
		else
			wallMove();
	}

	// Sends a random movement message to the robot
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

	// Sends a wall following movement message to the robot
	void wallMove() {

		// update the robot state
		this->state = wall;

		// Create the message. 
		geometry_msgs::Twist out_msg;

		// left or right laser value
		float laser_min;

		// signal: -1 if left laser, 1 if right laser; so that the right turns are made
		int signal;

		// get the laser to use and the corresponding signal
		getLaser(laser_min, signal);

		// determine what to do according to the laser value
		// if the laser distance is close to the intended wall distance, go forward
		if (laser_min >= this->distance - MIN_ERROR && laser_min <= this->distance + MIN_ERROR) {
			out_msg.linear.x = 1;
			out_msg.angular.z = 0;
		}
		// if the laser distance is a lot smaller than the intended wall distance, turn hard
		// left if the right laser is being used, right if the left laser is being used
		// in order to get away from the wall
		else if (laser_min < this->distance - MAX_ERROR) {
			out_msg.linear.x = 0.1;
			out_msg.angular.z = signal * M_PI / 4;
		}
		// if the laser distance is smaller than the intended wall distance, turn
		// left if the right laser is being used, right if the left laser is being used
		// in order to get away from the wall
		else if (laser_min < this->distance - MIN_ERROR) {
			out_msg.linear.x = 1;
			out_msg.angular.z = signal * M_PI / 8;
		}
		// if the laser distance is a lot larger than the intended wall distance, turn hard
		// right if the right laser is being used, left if the left laser is being used
		// in order to get closer to the wall
		else if (laser_min > this->distance + MAX_ERROR) {
			out_msg.linear.x = 0.5;
			out_msg.angular.z = - signal * M_PI / 4;
		}
		// if the laser distance is larger than the intended wall distance, turn
		// right if the right laser is being used, left if the left laser is being used
		// in order to get closer to the wall
		else if (laser_min > this->distance + MIN_ERROR) {
			out_msg.linear.x = 1;
			out_msg.angular.z = - signal * M_PI / 8;
		}
		// this should not happen, but just in case, do nothing and go back to random movements
		else {
			out_msg.linear.x = 0;
			out_msg.angular.z = 0;
			this->state = random;
			this->laser = unknown;
		}
		this->pub.publish(out_msg);
		ROS_INFO_STREAM("Velocity: "	<< " linear=" << out_msg.linear.x << " angular=" << out_msg.angular.z);
	}

	// Determines which laser to use, the right or the left, and the corresponding turn signal
	void getLaser(float& laser_min, int& signal) {
		// if only right laser sees wall, use right laser
		if (this->left == FLT_MAX && this->right < FLT_MAX) {
			laser_min = this->right;
			this->laser = right_laser;
			signal = 1;
		}
		// if only left laser sees wall, use left laser
		else if (this->right == FLT_MAX && this->left < FLT_MAX) {
			laser_min = this->left;
			this->laser = left_laser;
			signal = -1;
		}
		// if right laser was previously chosen, keep using it
		else if (this->laser == right_laser) {
			laser_min = this->right;
			signal = 1;						
		}
		// if left laser was previously chosen, keep using it
		else if (this->laser == left_laser) {
			laser_min = this->left;
			signal = -1;						
		}
		// if no laser was previously chosen and right laser is closer, use right laser
		else if (this->right < this->left) {
			laser_min = this->right;
			this->laser = right_laser;
			signal = 1;
		}
		// if no laser was previously chosen and right laser is not closer, use left laser
		else {
			laser_min = this->left;
			this->laser = left_laser;
			signal = -1;			
		}
	}

};

int main(int argc, char **argv) {
	
	// Seed the random number generator.
	srand((unsigned) time(NULL) * getpid());

	// Initialize the ROS system and become a node.
	ros::init(argc, argv, "publish_velocity");

	// Create an object of the Wall class that will take care of everything.
	string name(argv[1]);
	float distance = atof(argv[2]);
	Wall *wall = new Wall(name, distance);

	// Let ROS take over.
	ros::spin();

}