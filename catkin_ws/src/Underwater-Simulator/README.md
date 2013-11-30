underwater-simulator
====================

Underwater simulator built on top of Gazebo to simulate a pool environment for tasks for the RoboSub 2014 international competition.

INSTALLATION INSTRUCTIONS:

1. 	Gazebo Installation and Gazebo-ROS Packages: http://gazebosim.org/wiki/Tutorials/1.9/Installing_gazebo_ros_Packages

	NOTE: This page provides external links to installing ROS Hydro and Gazebo as a stand-alone.
		  Then it explains how to install the Gazebo-ROS Packages.
		  Follow all the instructions carefully. If you have already have ROS Hydro, skip that part.

2.	Clone the Underwater-Simulator in your catkin workspace and run: $catkin_make

	In our case, the package is already in a catkin workspace! So just pull "McGill_RoboSub_2014/catkin_ws"
	Run: $source devel/setup.bash
	
	$catkin_make

3.	Run: $roslaunch Underwater-Simulator Underwater-Simulator.launch
	
	If it does not run, try sourcing setup.bash again and re-run the simulator.

RECOMMENDED READING: http://gazebosim.org/wiki/Tutorials/1.9/Creating_ROS_plugins_for_Gazebo

Underwater-Simulator Directory Structure:
	launch/

		contains .launch files which launch an environment based on a world file
	worlds/

		contains .world files which are used by the launch files
	models/

		contains the models.
		DO NOT CHANGE THE MODEL NAMES! 
	src/

		contains plugins


There is a test package "robot_move_test" in the workspace. This can be used to pass a twist message to the robot and 
move it in the simulator. Run: "rosrun robot_move_test robot_move_test"
