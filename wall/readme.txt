# Reactive Robot

## Authors: Ângela Cardoso and Inês Caldas

Project developed for the ROBO curricular unit, MIEIC, FEUP.

Wall following reactive robot using ROS and STDR_Simulator.

### Installation instructions:

1. Configure a catkin workspace, let's call it catkin_ws
2. Copy the wall folder to catkin_ws/src
3. Build the catkin workspace: catkin build
4. Setup environment variables: . devel/setup.bash
5. Launch one of the available launchers:

          * roslaunch wall wall_DD.launch
          * roslaunch wall wall_inD.launch
          * roslaunch wall wall_outD.launch
          * roslaunch wall wall_StarStar.launch
          * roslaunch wall wall_inStar.launch
          * roslaunch wall wall_outStar.launch
          
5. To custom configurations change the respective file in wall/resources


### Requirements:

1. ROS Kinetic
2. STDR Simulator
