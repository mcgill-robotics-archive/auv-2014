underwater-simulator
====================

Underwater simulator built on top of Gazebo to simulate a pool environment for tasks for the RoboSub 2014 international competition.

Run: 
	roscore &
	roslaunch Underwater-Simulator Underwater-Simulator.launch

Directory Structure:
	launch/
		contains .launch files which launch an environment based on a world file
	worlds/
		contains .world files which are used by the launch files
	models/
		contains the models
	src/ 
		contains plugins
