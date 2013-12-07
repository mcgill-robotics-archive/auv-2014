
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
	partial_cmd_vel - from front-end
	zdes - from front-end
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
#include "std_msgs/Float64.h"
#include "depthController.h"

// using namespace std; //what does this do?!?!

//global vars
float zdes = 0;
float zest = 0;

input_xdot = 0;
input_ydot = 0;
input_zdot = 0;
input_alphadot = 0;
input_betadot = 0;
input_gammadot = 0;

void partial_cmd_vel_callback(const geometry_msgs::Twist twistMsg)
{
	//from frontend, missing z velocity for now
	ROS_INFO("Subscriber received twist");
	input_xdot = twistMsg.data.linear.x;
	input_ydot = twistMsg.data.linear.y;
	input_zdot = twistMsg.data.linear.z;
	input_alphadot = twistMsg.data.angular.x;
	input_betadot = twistMsg.data.angular.y;
	input_gammadot = twistMsg.data.angular.z;
}

void zdes_callback(const std_msgs::Float64 data)
{
	ROS_INFO("Subscriber received zdes");
	zdes = data.data;
}

void pose_callback(const geometry_msgs::pose data)
{
	ROS_INFO("Subscriber received zest");
	zest = data.position.z
}


int main(int argc, char **argv)
{
	//Parameters
	float m = 30; //mass in kg
	float g = 9.81;
	float cd = 0.1; //arbitrary drag coefficient
	float buoyancy = 0.02; %percent buoyancy
	float dt = 0.01 //temporary! TODO update this dynamically

	//initializations
	float zdot_old = 0; initial velocity
	float zdot_new = zdot_old;
	float Tz = 0;

	// ROS subscriber setup
	ros::init(argc,argv,"depthController");
	ros::NodeHandle n;
	ros::Subscriber partial_cmd_vel_subscriber = n.subscribe("partial_cmd_vel", 1000, partial_cmd_vel_callback);
	ros::Subscriber zdes_subscriber = n.subscribe("zdes", 1000, zdes_callback);
	ros::Subscriber pose_subscriber = n.subscribe("pose", 1000, pose_callback);


	ros::Rate loop_rate(100); //100 hz??
	
	bool ready = 0
	while (ready == 0)
	{
		ready = 1		 
		if (partial_cmd_vel_subscriber.getNumPublishers() == 0) ready = 0;
		if (zdes_subscriber.getNumPublishers() == 0) ready = 0;
		if (pose_subscriber.getNumPublishers() == 0) ready = 0;
	}

	while(ros::ok())
	{
		ros::spinOnce();
		
		// Position Controller
		Tz = 0.1*(z_des-z_est)

		//Dynamics simulator
		zdot_new = dt*(Tz/m - buoyancy*g - cd*zdot_old/m) + zdot_old;

		
		// publish here
		std::stringstream ss;
		ss << "Speed: " << zdot_new;
		ROS_INFO("%s", ss.str())


		

		loop_rate.sleep();
	}


	return 0;

}