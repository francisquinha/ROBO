// This program subscribes to turtle1/pose and turtle2/pose
// then it moves turtle3 in such a way that it follows the closest of the other two turtles.
#include <math.h>
#include <string.h>
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/SetPen.h>
using namespace std;

class Follow {

public:

	Follow() {

		// Initiate the publisher.
		pub = nh.advertise<geometry_msgs::Twist>("turtle3/cmd_vel", 1000);
		
		// Initiate the subscriber 1 - turtle1 position.
		sub1 = nh.subscribe("turtle1/pose", 1000, &Follow::turtle1pose, this);

		// Initiate the subscriber 2 - turtle2 position.
		sub2 = nh.subscribe("turtle2/pose", 1000, &Follow::turtle2pose, this);

		// Initiate the subscriber 3 - turtle3 position.
		sub3 = nh.subscribe("turtle3/pose", 1000, &Follow::turtle3pose, this);

	}
	
private:
	
	ros::NodeHandle nh;
	ros::Publisher pub;
	ros::Subscriber sub1;
	ros::Subscriber sub2;
	ros::Subscriber sub3;
	float x1;	// Turtle 1 position first coordinate.
	float y1;	// Turtle 1 position second coordinate.
	float x2;	// Turtle 2 position first coordinate.
	float y2;	// Turtle 2 position second coordinate.

	void turtle1pose(const turtlesim::Pose& msg) {
		this->x1 = msg.x;
		this->y1 = msg.y;		
	}

	void turtle2pose(const turtlesim::Pose& msg) {
		this->x2 = msg.x;
		this->y2 = msg.y;		
	}

	void turtle3pose(const turtlesim::Pose& msg) {

		// Create the outgoing message.
		geometry_msgs::Twist out_msg;

		// Compute linear and angular velocities and fill in the outgoing message.
		// The other four fields, which are ignore by turtlesim, default to 0.		
		float dist1 = sqrt((this->x1 - msg.x) * (this->x1 - msg.x) 
			+ (this->y1 - msg.y) * (this->y1 - msg.y));
		float dist2 = sqrt((this->x2 - msg.x) * (this->x2 - msg.x) 
			+ (this->y2 - msg.y) * (this->y2 - msg.y));

		// Linear velocity is given by the distance to the closest turtle minus 1.
		// Angular velocity is given by the difference between the turtles current angle
		// and the angle of the vector it forms with the closest turtle.
		if (dist1 <= dist2) {
			out_msg.angular.z = remainder(atan2(this->y1 - msg.y, this->x1 - msg.x) 
				- msg.theta, 2 * M_PI);
			if (dist1 > 1)
				out_msg.linear.x = dist1 - 1;
		}
		else {
			out_msg.angular.z = remainder(atan2(this->y2 - msg.y, this->x2 - msg.x) 
				- msg.theta, 2 * M_PI);
			if (dist2 > 1)
				out_msg.linear.x = dist2 - 1;
		}

		// Publish the message.
		pub.publish(out_msg);

		// Send a message to rosout with the details.
		ROS_INFO_STREAM("Sending follow command:" 
			<< " linear=" << out_msg.linear.x << " angular=" << out_msg.angular.z);

	}

};


int main(int argc, char **argv) {

	// Initialize the ROS system and become a node.
	ros::init(argc, argv, "follow_turtle");

	// Create an object of the Follow class that will take care of everything.
	Follow follow;

	// Let ROS take over.
	ros::spin();

}
