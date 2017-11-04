# ROBO

Course work for the Robotics class at MIEIC - FEUP

## Wall Following Robot

### Useful links

1. [STDR Simulator - Tutorials](http://wiki.ros.org/stdr_simulator/Tutorials);
2. [STDR - Creating a Robot](http://wiki.ros.org/stdr_simulator/Tutorials/Using%20YAML%20files#Creating_a_robot_with_the_two_laser_sensors);

### Useful commands

1. Start server: 
```shell
roslaunch stdr_launchers server_no_map.launch
```
2. Start gui:
```shell
roslaunch stdr_gui stdr_gui.launch
```
3. Start rviz: 
```shell
rosrun rviz rviz -d ~/catkin_ws/src/stdr/config.rviz
```
4. Load map: 
```shell
rosrun stdr_server load_map DD.yaml
```
5. Add robot: 
```shell
rosrun stdr_robot robot_handler add robot.xml
```
