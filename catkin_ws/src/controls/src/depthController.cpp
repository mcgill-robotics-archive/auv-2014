
/*
This is the first attempt at integrating a control system into the robosub simulation.
It subscribes to desired and estimated velocity topics and publishes a new velocity topic according to the dynamics of the vehicle.

Written by Nick Speal Dec 3.
*/

/*
Roadmap

Header stuff
Subscribers
	actual velocity - from simulator
	cmd_vel - (incomplete) from front-end
	d_des - from front-end
	pose of robot - from simulator
Position Controller
	computes thrust based on depth error
Dynamics simulator
	computes velocity based on all forces, including thrust
publishers
	cmd_vel - merge 5-DOF from frontend with z_vel from here


*/
#include "ros/ros.h"
#include 

// Subscribers

//Position Controller

//Dynamics simulator

//Publishers