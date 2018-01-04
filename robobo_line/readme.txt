# Robobo Line Follower

## Author: Ângela Cardoso

Project developed for the ROBO curricular unit, MIEIC, FEUP.

Line following reactive robot using ROS and Robobo.

### Assumptions / Requirements

1. The operating system is either Ubuntu 16 or some flavor of this.
2. ROS Kinetic is installed.
3. OpenCV version 3.3.1 is installed.
4. Catkin tools is installed (this is not really necessary, but the instructions that follow assume it is done).

### Installation instructions

1. Configure a catkin workspace, let's call it catkin_ws and assume it is in your home folder.
2. Copy the robobo_line and the com_mytechia_robobo_ros_msgs folders to ~/catkin_ws/src.
3. Go to the catkin_ws folder: cd ~/catkin_ws.
4. Build the catkin workspace: catkin build.
5. Setup environment variables: source ~/catkin_ws/devel/setup.bash.
6. Connect the Robobo Developer app and take note of the IP address provided.
7. Set up the ROS master URI using that IP address: export ROS_MASTER_URI=http://<IP address>:11311.
8. Run the command: rostopic echo /status.
9. Manually clockwise turn the Robobo phone holder all the way.
10. Check that the status changed and that PAN is now set to around 345 degrees.
11. Manually lift the Robobo phone holder to about 130 degrees, verifying in the status.
12. Place the phone in its spot and place the robobo on top of the line you wish it to follow.
13. Launch one of the available launchers:
	* roslaunch robobo_line red.launch;
    * roslaunch robobo_line green.launch;
    * roslaunch robobo_line blue.launch;
    * roslaunch robobo_line black.launch.
14. If necessary, change the arguments in these files:
	* color: 0 - black, 1 - red, 2 - green, 3 - blue;
	* kp: integer, proportional constant for PID controller;
	* ki: integer, integral constant for PID controller;
	* kd: integer, derivative constant for PID controller;
	* speed: integer, linear velocity of the robot;
	* distance: float between 0.0 and 0.8, 0.0 means the Region of Interest (ROI) is furthest from the robot and 0.8 means it is closest.
