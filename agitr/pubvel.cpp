// This program publishes randomly-generated velocity messages for a turtlesim turtle.
// If the turtle hits a wall it tries to move away from it by turning M_PI / 2 or -M_PI / 2.
#include <ros/ros.h>

// For geometry_msgs::Twist 
#include <geometry_msgs/Twist.h>

// For rand() and RAND_MAX
#include <stdlib.h>

#include <turtlesim/Pose.h>
#include <math.h>
#include <string.h>

using namespace std;

class PubVel {

public:
	PubVel(string name) {

		this->name = name;

		// Initiate the publisher.
		pub = nh.advertise<geometry_msgs::Twist>(name + "/cmd_vel", 1000);

		// Initiate the subscriber.
		// We subscribe to the turtle's position to detect walls.
		sub = nh.subscribe(name + "/pose", 1000, &PubVel::turtle_pose, this);

		this->signal = (rand() % 2) * 2 - 1;;
		this->counter = 0;
		this->next = (rand() % 10 + 1) * 10;
		
	}

private:
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Subscriber sub;
	int signal; // Determines whether the turtle rotates counterclockwise or clockwise.
	int counter; // Incremented in each turtle movement published message.
	int next; // When to reset the counter and randomize the signal.
	string name; // Name of the turtle to move.


	// A callback function, to receive the turtle position and move semi randomly.
	void turtle_pose(const turtlesim::Pose& msg) {

		this->counter++;	

		// Whenever the counter reaches next, reset it, randomize the signal 
		// and randomize the next check.
		if (this->counter == this->next) {
			this->counter = 0;
			this->signal = (rand() % 2) * 2 - 1;
			this->next = (rand() % 10 + 1) * 10;
		}

		// Create and fill in the message. 
		// The other four fields, which are ignore by turtlesim, default to 0.
		geometry_msgs::Twist out_msg;
		out_msg.linear.x = double(rand())/double(RAND_MAX);
		out_msg.angular.z = this->signal * double(rand())/double(RAND_MAX);

		if (msg.x <= 0 || msg.y <= 0 || msg.x >= 11 || msg.y >= 11)
			out_msg.angular.z = this->signal * M_PI / 2 ;

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

	// Create an object of the PubVel class that will take care of everything.
	string name(argv[1]);
	PubVel *pubvel = new PubVel(name);

	// Let ROS take over.
	ros::spin();

}