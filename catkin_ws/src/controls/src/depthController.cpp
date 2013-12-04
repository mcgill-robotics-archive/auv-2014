
/*
This is the first attempt at integrating a control system into the robosub simulation.
It subscribes to desired and estimated velocity topics and publishes a new velocity topic according to the dynamics of the vehicle.

Handwritten notes in page 79 of Nick's design notebook for P&C.

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
#include "geometry_msgs/Twist.h"
#include "depthController.h"

void cmdVelCallback(const geometry_msgs::Twist twistMsg)
{
	ROS_INFO("Subscriber received twist");
}
int main(int argc, char **argv)
{
	ros::init(argc,argv,"depthController");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/gazebo/robot_twist", 1000, cmdVelCallback);
	ros::spin();
	return 0;

}




// Subscribers

//Position Controller

//Dynamics simulator

//Publishers
