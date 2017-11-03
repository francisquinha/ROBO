// This program changes a turtle pen by calling the appropriate service.

#include <stdlib.h>
#include <string.h>
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <turtlesim/SetPen.h>
using namespace std;

class Change {

public:

	Change(string name, int r, int g, int b, int width, int off) {

		this->name = name;
		this->r = r;
		this->g = g;
		this->b = b;
		this->width = width;
		this->off = off;

		// Initiate the subscriber.
		// We subscribe to the turtle position, 
		// to make sure it exists before trying to change its pen.
		sub = nh.subscribe(name + "/pose", 1000, &Change::change_pen, this);

	}
	
private:
	
	ros::NodeHandle nh;
	ros::Subscriber sub;
	string name; // Turtle name.
	int r;	// Pen color red 0-255.
	int g;	// Pen color green 0-255.
	int b;	// Pen color blue 0-255.
	int width; // Pen width.
	int off; // Turn the pen off - 1, or keep it on - 0.

	void change_pen(const turtlesim::Pose& msg) {

		// Create a client object for the set_pen service.
		// This is used to set the pen according to the input arguments.
		ros::ServiceClient penClient = nh.serviceClient<turtlesim::SetPen>(name + "/set_pen");

		// Create the request and response objects.
		turtlesim::SetPen::Request pen_req;
		turtlesim::SetPen::Response pen_resp;

		pen_req.r = this->r;
		pen_req.g = this->g;
		pen_req.b = this->b;
		pen_req.width = this->width;
		pen_req.off = this->off;

		// Actually call the service.
		// This won't return until the service is complete.
		bool pen_success = penClient.call(pen_req, pen_resp);

		// Check for success and use the response.
		if (pen_success) 
			ROS_INFO_STREAM("Changed the pen of " << name << ".");
		else
			ROS_ERROR_STREAM("Failed to change the pen of " << name<< ".");

		// No need to keep this node running, its job is done.
		ros::shutdown();
	}

};

int main(int argc, char **argv) {

	// Initialize the ROS system and become a node.
	ros::init(argc, argv, "change_turtle_pen");

	// Create an object of the Change class that will take care of everything.
	string name(argv[1]);
	int r = atoi(argv[2]);
	int g = atoi(argv[3]);
	int b = atoi(argv[4]);
	int width = atoi(argv[5]);
	int off = atoi(argv[6]);
	Change *change = new Change(name, r, g, b, width, off);

	// Let ROS take over.
	ros::spin();

}
