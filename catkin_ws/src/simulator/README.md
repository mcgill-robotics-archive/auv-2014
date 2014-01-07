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

Bin Task:

Run: $roslaunch simulator bin_task.launch

This will create a world with robot and the 4 bins aligned.

To subscribe to the image from the camera pointing downwards:
Make sure your ROS node subscribes to "/my_robot/camera_down/image_raw"

The topics published by the camera are:

/my_robot/camera_down/camera_info
/my_robot/camera_down/image_raw
/my_robot/camera_down/image_raw/compressed
/my_robot/camera_down/image_raw/compressed/parameter_descriptions
/my_robot/camera_down/image_raw/compressed/parameter_updates
/my_robot/camera_down/image_raw/compressedDepth
/my_robot/camera_down/image_raw/compressedDepth/parameter_descriptions
/my_robot/camera_down/image_raw/compressedDepth/parameter_updates
/my_robot/camera_down/image_raw/theora
/my_robot/camera_down/image_raw/theora/parameter_descriptions
/my_robot/camera_down/image_raw/theora/parameter_updates
/my_robot/camera_down/parameter_descriptions
/my_robot/camera_down/parameter_updates


To move the robot slowly so it hovers over the bins, you will need to use
a PS3 controller and the front_end package OR an easier way is to use
the robot_move_model test package. 

Run: $rosrun robot_move_model robot_move_model
Pass it x:0, y:0.05, z:0

The robot will move over the bins and you can see if your filters are working.

A negative z should lower the robot, in case that's needed.

-----------------------------------------------------------------
