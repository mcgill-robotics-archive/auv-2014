
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
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64.h"
#include "depthController.h"
#include "gazebo_msgs/ModelStates.h"

// using namespace std; //what does this do?!?!

//global vars
float z_des = 0;
float z_est = 0;

float input_xdot = 0;
float input_ydot = 0;
float input_zdot = 0;
float input_alphadot = 0;
float input_betadot = 0;
float input_gammadot = 0;

void partial_cmd_vel_callback(const geometry_msgs::Twist twistMsg)
{
	//from frontend, missing z velocity for now
	ROS_DEBUG("Subscriber received twist");
	input_xdot = twistMsg.linear.x;
	input_ydot = twistMsg.linear.y;
	input_zdot = twistMsg.linear.z;
	input_alphadot = twistMsg.angular.x;
	input_betadot = twistMsg.angular.y;
	input_gammadot = twistMsg.angular.z;                
}

void zdes_callback(const std_msgs::Float64 data)
{
	ROS_DEBUG("Subscriber received zdes");
	z_des = data.data;
	ROS_DEBUG("zdes: %f", z_des);
}

void pose_callback(const gazebo_msgs::ModelStates data)

{
	// Pose contains the pose of all the models in the simulator. Grab the pose of the first model with [0]
	ROS_DEBUG("Subscriber received zest");
    z_est = data.pose[0].position.z;
    ROS_DEBUG("z_est: %f", z_est);
}


int main(int argc, char **argv)
{
	//Parameters
	float m = 30; //mass in kg
	float g = 9.81;
	float cd = 0.1; //arbitrary drag coefficient
	float buoyancy = 0.02; // %percent buoyancy
	float dt = 0.01; //temporary! TODO update this dynamically
	float kp = 1000; //proportional controller
        float ki = 100;

	//initializations
	float zdot_old = 0; //initial velocity
	float zdot_new = 0;
	float Tz = 0;
        float ep = 0; //error
        float ei = 0; //integral error

	// ROS subscriber setup
	ros::init(argc,argv,"depthController");
	ros::NodeHandle n;
	ros::Subscriber partial_cmd_vel_subscriber = n.subscribe("partial_cmd_vel", 1000, partial_cmd_vel_callback);
	ros::Subscriber zdes_subscriber = n.subscribe("zdes", 1000, zdes_callback);
	ros::Subscriber pose_subscriber = n.subscribe("gazebo/model_states", 1000, pose_callback);
	//add clock subscription

	//ROS Publisher setup
	ros::Publisher cmd_vel_publisher = n.advertise<geometry_msgs::Twist>("/gazebo/robot_twist", 100);
	geometry_msgs::Twist twistMsg; //define variable

	ros::Rate loop_rate(1/dt); //100 hz??
	
	bool ready = 0;
	ROS_INFO("DepthController waiting for all subscribers to have content...");
	while (ready == 0)
	{
	       
		ROS_DEBUG_THROTTLE(2,"Waiting...");
		ready = 1;		 
		if (partial_cmd_vel_subscriber.getNumPublishers() == 0) {ready = 0;}
		else {ROS_DEBUG_THROTTLE(2,"got partial cmd vel");}
		if (zdes_subscriber.getNumPublishers() == 0) {ready = 0;}
		else {ROS_DEBUG_THROTTLE(2,"got zdes");}
		if (pose_subscriber.getNumPublishers() == 0) {ready = 0;}
		else {ROS_DEBUG_THROTTLE(2,"got pose");}
	}

	ROS_INFO("All Subscribers Live. Starting Controller!");
	while(ros::ok())
	{
		ros::spinOnce();	//is it a problem to have this at the top not the bottom?
		
		// Position Controller
                ep = z_des-z_est;
                ei += ep*dt; //integral error
		Tz = kp*ep + ki*ei;

		//Dynamics simulator
		zdot_new = dt*(Tz/m - buoyancy*g - cd*zdot_old/m) + zdot_old;	//TODO update dt dynamically with clock

		//put cmd_together
		twistMsg.linear.x = input_xdot;		//unchanged from receipt
		twistMsg.linear.y = input_ydot;		//unchanged from receipt
		twistMsg.linear.z = zdot_new;		
		twistMsg.angular.x = input_alphadot;//unchanged from receipt
		twistMsg.angular.y = input_betadot;	//unchanged from receipt
		twistMsg.angular.z = input_gammadot;//unchanged from receipt   		


		// publish here
		//std::stringstream ss;
		//ss << "Speed: " << zdot_new;
		//ROS_DEBUG("%s", ss.str());
        ROS_DEBUG("zdot_new: %f", zdot_new);

		cmd_vel_publisher.publish(twistMsg);
		loop_rate.sleep();
	}


	return 0;

}
