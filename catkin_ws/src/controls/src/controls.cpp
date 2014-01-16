
/*

First integrated 5DOF control system
Subscribes to setpoints and estimated states and publishes a net wrench to minimize error.
Open Loop Speed Control.	

Written by Nick Speal Jan 10.
*/


/*
Topics in form topic messagetype

Subscribers:


Publishers:

/controls/wrench geometry_msgs/Wrench

*/



/*
Roadmap



*/
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"
#include "std_msgs/Float64.h"
#include "gazebo_msgs/ModelStates.h"

#include "planner/setPoints.h"	
#include "planner/ValueControl.h" //useless? / deprecated?

// using namespace std; //what does this do?!?!

//global vars
float z_des = 0;
float z_est = 0;

float setPoint_XPos = 0;
float setPoint_YPos = 0;
float setPoint_Depth = 0;
float setPoint_Yaw = 0;
float setPoint_Pitch = 0;
float setPoint_XSpeed = 0;
float setPoint_YSpeed = 0;
float setPoint_YawSpeed = 0;

bool isActive_XPos = 0;	//expecting int, but I'd rather bool. Is this valid casting?
bool isActive_YPos = 0;
bool isActive_Depth = 0;
bool isActive_Yaw = 0;
bool isActive_Pitch = 0;
bool isActive_XSpeed = 0;
bool isActive_YSpeed = 0;
bool isActive_YawSpeed = 0;

float estimated_XPos = 0;
float estimated_YPos = 0;
float estimated_Depth = 0;
float estimated_Pitch = 0;
float estimated_Yaw = 0;

void setPoints_callback(const planner::setPoints setPointsMsg)
{
	ROS_DEBUG("Subscriber received set points");
	setPoint_XPos = setPointsMsg.XPos.data;
	setPoint_YPos = setPointsMsg.YPos.data;
	setPoint_Depth = setPointsMsg.Depth.data;
	setPoint_Yaw = setPointsMsg.Yaw.data;
	setPoint_Pitch = setPointsMsg.Pitch.data;
	setPoint_XSpeed = setPointsMsg.XSpeed.data;
	setPoint_YSpeed = setPointsMsg.YSpeed.data;
	setPoint_YawSpeed = setPointsMsg.YawSpeed.data;

	isActive_XPos = setPointsMsg.XPos.isActive;
	isActive_YPos = setPointsMsg.YPos.isActive;
	isActive_Depth = setPointsMsg.Depth.isActive;
	isActive_Yaw = setPointsMsg.Yaw.isActive;
	isActive_Pitch = setPointsMsg.Pitch.isActive;
	isActive_XSpeed = setPointsMsg.XSpeed.isActive;
	isActive_YSpeed = setPointsMsg.YSpeed.isActive;
	isActive_YawSpeed = setPointsMsg.YawSpeed.isActive;

}

void estimatedState_callback(const gazebo_msgs::ModelStates data)

{
	// Pose contains the pose of all the models in the simulator. Grab the pose of the first model with [0]
	ROS_DEBUG("Subscriber received estimated data");
    
    estimated_XPos = data.pose[0].position.x;
    estimated_YPos = data.pose[0].position.y;
    estimated_Depth = data.pose[0].position.z;
    estimated_Pitch = 0; //hardcoded TODO FIX THIS
	estimated_Yaw = 0; //hardcoded TODO FIX THIS


    //still need to add pitch and yaw

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
	

    float ep_XPos = 0; //error
    float ei_XPos = 0; //integral error
    float ep_YPos = 0;
    float ei_YPos = 0;
    float ep_Depth = 0;
    float ei_Depth = 0;
    float ep_Pitch = 0;
    float ei_Pitch = 0;
    float ep_Yaw = 0;
    float ei_Yaw = 0;

    float Fx = 0;
    float Fy = 0;
    float Fz = 0;
    float Ty = 0;
    float Tz = 0;

    float OL_coef_x;	//set later in the controller
    float OL_coef_y;
    float OL_coef_yaw;

	// ROS subscriber setup
	ros::init(argc,argv,"controls");
	ros::NodeHandle n;
	ros::Subscriber setPoints_subscriber = n.subscribe("setPoints", 1000, setPoints_callback);
	ros::Subscriber estimatedState_subscriber = n.subscribe("gazebo/model_states", 1000, estimatedState_callback);
	//add clock subscription

	//ROS Publisher setup
	ros::Publisher wrench_publisher = n.advertise<geometry_msgs::Wrench>("/controls/wrench", 100);
	geometry_msgs::Wrench wrenchMsg; //define variable to publish

	ros::Rate loop_rate(1/dt); //100 hz??
	
	bool ready = 0;
	ROS_INFO("controls waiting for all subscribers to have content...");
	while (ready == 0)
	{
	       
		ROS_DEBUG_THROTTLE(2,"Waiting...");
		ready = 1;		 
		if (setPoints_subscriber.getNumPublishers() == 0) {ready = 0;}
		else {ROS_DEBUG_THROTTLE(2,"got setPoints");}
		if (estimatedState_subscriber.getNumPublishers() == 0) {ready = 0;}
		else {ROS_DEBUG_THROTTLE(2,"got estimated State");}
	}

	ROS_INFO("All Subscribers Live. Starting Controller!");
	while(ros::ok())
	{
		ros::spinOnce();	//is it a problem to have this at the top not the bottom?
		
		// Decoupled Controllers

		//zero unless otherwise specified
		Fx = 0;
    	Fy = 0;
    	Fz = 0;
    	Ty = 0;
    	Tz = 0;

		//X 
		if (isActive_XPos)
		{
			ep_XPos = setPoint_XPos - estimated_XPos;
			ei_XPos += ep_XPos*dt;
			Fx = kp*ep_XPos + ki*ei_XPos;
			//ROS_INFO("controlling xpos");
		}
        
        if (isActive_XSpeed)
        {
        	OL_coef_x = 1;
        	Fx = OL_coef_x*setPoint_XSpeed;
        	//ROS_INFO("controlling xspeed");
        }

        //Y 
		if (isActive_YPos)
		{
			ep_YPos = setPoint_YPos - estimated_YPos;
			ei_YPos += ep_YPos*dt;
			Fy = kp*ep_YPos + ki*ei_YPos;
		}
        
        if (isActive_YSpeed)
        {
        	OL_coef_y = 1;
        	Fy = OL_coef_y*setPoint_YSpeed;
        }

        //Z 
		if (isActive_Depth)
		{
			ep_Depth = setPoint_Depth - estimated_Depth;
			ei_Depth += ep_Depth*dt;
			//Fz = kp*ep_Depth + ki*ei_Depth;
			Fz = 0; // workaround temp
			//Fz += buoyancy*m*g; //Account for positive buoyancy bias
		}

		//Pitch
		if (isActive_Pitch)
		{
			ep_Pitch = setPoint_Pitch - estimated_Pitch;
			ei_Pitch += ep_Pitch*dt;
			Ty = kp*ep_Pitch + ki*ei_Pitch;
		}

		//Yaw
		if (isActive_Yaw)
		{
			ep_Yaw = setPoint_Yaw - estimated_Yaw;
			ei_Yaw += ep_Yaw*dt;
			Tz = kp*ep_Yaw + ki*ei_Yaw;
		}

		if (isActive_YawSpeed)
        {
        	OL_coef_yaw = 1;
        	Ty = OL_coef_yaw*setPoint_YawSpeed;
        }

		// Assemble Wrench
		wrenchMsg.force.x = Fx;
		wrenchMsg.force.y = Fy;
		wrenchMsg.force.z = Fz;
		wrenchMsg.torque.x = 0;	//no active roll control
		wrenchMsg.torque.y = Ty;
		wrenchMsg.torque.z = Tz;

		wrench_publisher.publish(wrenchMsg);
		loop_rate.sleep();


	}
	return 0;
}
