// This program spawns a new turtlesim turtle by calling the appropriate service.

#include <stdlib.h>
#include <string.h>
#include <ros/ros.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
using namespace std;

class Spawn {

public:

	Spawn(string name, float x, float y, float theta) {

		this->name = name;
		this->x = x;
		this->y = y;
		this->theta = theta;

		// Initiate the subscriber.
		// We subscribe to turtle1 position, to make sure that turtlesim has been launched
		// and that it's ok to spawn a new turtle.
		sub = nh.subscribe("turtle1/pose", 1000, &Spawn::spawn_turtle, this);

	}
	
private:
	
	ros::NodeHandle nh;
	ros::Subscriber sub;
	string name; // Turtle name.
	float x; // Turtle position first coordinate.
	float y; // Turtle position second coordinate.
	float theta; // Turtle orientation angle.

	void spawn_turtle(const turtlesim::Pose& msg) {

		// Create a client object for the spawn service. 
		// This needs to know the data type of the service and its name.
		ros::ServiceClient spawnClient = this->nh.serviceClient<turtlesim::Spawn>("spawn");

		// Create the request and response objects.
		turtlesim::Spawn::Request req;
		turtlesim::Spawn::Response resp;

		// Fill in the request data members.
		req.name = this->name;
		req.x = this->x;
		req.y = this->y;
		req.theta = this->theta;

		// Actually call the service.
		// This won't return until the service is complete.
		bool success = spawnClient.call(req, resp);

		// Check for success and use the response.
		if (success)
			ROS_INFO_STREAM("Spawned a turtle named " << resp.name << ".");
		else
			ROS_ERROR_STREAM("Failed to spawn.");
		
		// No need to keep this node running, its job is done.
		ros::shutdown();
	}

};

int main(int argc, char **argv) {

	// Initialize the ROS system and become a node.
	ros::init(argc, argv, "spawn_turtle");

	// Create an object of the Spawn class that will take care of everything.
	string name(argv[1]);
	float x = atof(argv[2]);
	float y = atof(argv[3]);
	float theta = atof(argv[4]);
	Spawn *spawn = new Spawn(name, x, y, theta);

	// Let ROS take over.
	ros::spin();

}
