simulator
====================

Underwater simulator built on top of Gazebo to simulate a pool environment for tasks for the RoboSub 2014 international competition.

INSTALLATION INSTRUCTIONS:

1. 	Gazebo Installation and Gazebo-ROS Packages: http://gazebosim.org/wiki/Tutorials/1.9/Installing_gazebo_ros_Packages

	NOTE: This page provides external links to installing ROS Hydro and Gazebo as a stand-alone.
		  Then it explains how to install the Gazebo-ROS Packages.
		  Follow all the instructions carefully. If you have already have ROS Hydro, skip that part.

2.	Clone the simulator in your catkin workspace and run: $catkin_make

	In our case, the package is already in a catkin workspace! So just pull "McGill_RoboSub_2014/catkin_ws"
	
	Run: $source devel/setup.bash
	
	$catkin_make


	Note: If you don't want to source your Catkin workspace everytime you want to run ROS, you can add the sourcing
	step in your ~/.bashrc file. This file is executed whenever ROS starts when you log into your Ubuntu session.
	Under is an example of this:

	source /opt/ros/hydro/setup.bash
	source ~/Git_Repositories/McGill_RoboSub_2014/catkin_ws/devel/setup.bash

3.	Run: $roslaunch simulator simulator.launch
	
	If it does not run, try sourcing setup.bash again and re-run the simulator.

RECOMMENDED READING: http://gazebosim.org/wiki/Tutorials/1.9/Creating_ROS_plugins_for_Gazebo

simulator Directory Structure:
	
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

-----------------------------------------------------------------
topics:
/controls/wrench
/gazebo/simulator/robot_twist
/gazebo/simulator/thruster_forces
/simulator/camera1/image_raw
/simulator/camera2/image_raw
/simulator/camera3/image_raw
