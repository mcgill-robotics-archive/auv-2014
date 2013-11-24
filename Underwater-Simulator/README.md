underwater-simulator
====================

Underwater simulator built on top of Gazebo to simulate a pool environment for tasks for the RoboSub 2014 international competition.

A catkin workspace is needed. Please read this tutorial before setting up your workspace:
http://gazebosim.org/wiki/Tutorials/1.9/Creating_ROS_plugins_for_Gazebo

Run:

	roscore &

	roslaunch Underwater-Simulator Underwater-Simulator.launch

Directory Structure:

	launch/

		contains .launch files which launch an environment based on a world file

	worlds/

		contains .world files which are used by the launch files

	models/

		contains the models. Download models from google drive after
		cloning the repository.
		Models:
		- bin.dae
		- gate.dae
		- torpedo.dae
		- torpedoTargets.dae
		- wheelTask.dae
		- robot_1.0.dae

	src/

		contains plugins
