
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
TODO
service for reset
ros params
	implement check if not parametrized properly (rather than 0 default which can go unnoticed)

/*



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
double z_des = 0;
double z_est = 0;

double setPoint_XPos; //initialized by ros params
double setPoint_YPos;
double setPoint_Depth;
double setPoint_Yaw;
double setPoint_Pitch;
double setPoint_XSpeed;
double setPoint_YSpeed;
double setPoint_YawSpeed;

int8_t isActive_XPos;	
int8_t isActive_YPos;
int8_t isActive_Depth;
int8_t isActive_Yaw;
int8_t isActive_Pitch;
int8_t isActive_XSpeed;
int8_t isActive_YSpeed;
int8_t isActive_YawSpeed;

double estimated_XPos = 0;
double estimated_YPos = 0;
double estimated_Depth = 0;
double estimated_Pitch = 0;
double estimated_Yaw = 0;

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
	// Pose contains the pose of all the models in the simulator. Grab the pose of the first model (robot as opposed to buoys etc) with [0]
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
	//Create ROS Node
	ros::init(argc,argv,"controls");
	ros::NodeHandle n;

	//Parameters
	double m; //mass in kg
	double g;
	double cd; // drag coefficient
	double buoyancy; // %percent buoyancy
	double dt; //temporary! TODO update this dynamically
	double kp; //proportional controller
    double ki; //integral
    double kd; //derivative

    //ROS Params

    n.param<double>("gains/kp", kp, 0.0);
    n.param<double>("gains/ki", ki, 0.0);
    n.param<double>("gains/kd", kd, 0.0);

    n.param<double>("setPoints/XPos/value", setPoint_XPos, 0.0);
    n.param<double>("setPoints/XSpeed/value", setPoint_XSpeed, 0.0);

    n.param<double>("setPoints/YPos/value", setPoint_YPos, 0.0);
    n.param<double>("setPoints/YSpeed/value", setPoint_YSpeed, 0.0);

    n.param<double>("setPoints/Depth/value", setPoint_Depth, 0.0);

    n.param<double>("setPoints/Yaw/value", setPoint_Yaw, 0.0);
    n.param<double>("setPoints/YawSpeed/value", setPoint_YawSpeed, 0.0);
/*
TODO figure this out - doesnt compile

    n.param<int8_t>("setPoints/XPos/isActive", isActive_XPos, 0);
    n.param<int8_t>("setPoints/XSpeed/isActive", isActive_XSpeed, 0);

    n.param<int8_t>("setPoints/YPos/isActive", isActive_YPos, 0);
    n.param<int8_t>("setPoints/YSpeed/isActive", isActive_YSpeed, 0);

    n.param<int8_t>("setPoints/Depth/isActive", isActive_Depth, 0);

    n.param<int8_t>("setPoints/Yaw/isActive", isActive_Yaw, 0);
    n.param<int8_t>("setPoints/YawSpeed/isActive", isActive_YawSpeed, 0);
*/
    n.param<double>("coefs/mass", m, 30.0);
    n.param<double>("coefs/buoyancy", buoyancy, 0.02);
    n.param<double>("coefs/drag", cd, 0.0);
    n.param<double>("coefs/gravity", g, 9.81);
    n.param<double>("coefs/dt", dt, 0.1);


	//initializations
	

    double ep_XPos = 0; //error
    double ei_XPos = 0; //integral error
    double ed_XPos = 0; //derivative error
    double ep_XPos_prev = 0; //proportional error at last timestep
    double ep_YPos = 0;
    double ei_YPos = 0;
    double ed_YPos = 0;
    double ep_YPos_prev = 0;
    double ep_Depth = 0;
    double ei_Depth = 0;
    double ed_Depth = 0;
    double ep_Depth_prev = 0;
    double ep_Pitch = 0;
    double ei_Pitch = 0;
    double ed_Pitch = 0;
    double ep_Pitch_prev = 0;
    double ep_Yaw = 0;
    double ei_Yaw = 0;
    double ed_Yaw = 0;
    double ep_Yaw_prev = 0;


    double Fx = 0;
    double Fy = 0;
    double Fz = 0;
    double Ty = 0;
    double Tz = 0;

    double OL_coef_x;	//set later in the controller
    double OL_coef_y;
    double OL_coef_yaw;

	// ROS subscriber setup
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
			ep_XPos_prev = ep_XPos;
			ep_XPos = setPoint_XPos - estimated_XPos;
			ei_XPos += ep_XPos*dt;
			ed_XPos = (ep_XPos - ep_XPos_prev)/dt;
			Fx = kp*ep_XPos + ki*ei_XPos + kd*ed_XPos;
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
			ep_YPos_prev = ep_YPos;
			ep_YPos = setPoint_YPos - estimated_YPos;
			ei_YPos += ep_YPos*dt;
			ed_YPos = (ep_YPos - ep_YPos_prev)/dt;
			Fy = kp*ep_YPos + ki*ei_YPos + kd*ed_YPos;
		}
        
        if (isActive_YSpeed)
        {
        	OL_coef_y = 1;
        	Fy = OL_coef_y*setPoint_YSpeed;
        }

        //Z 
		if (isActive_Depth)
		{
			ep_Depth_prev = ep_Depth;
			ep_Depth = setPoint_Depth - estimated_Depth;
			ei_Depth += ep_Depth*dt;
			ed_Depth = (ep_Depth - ep_Depth_prev)/dt;
			Fz = kp*ep_Depth + ki*ei_Depth + kd*ed_Depth;
			//Fz = 0; // workaround temp
			//Fz += buoyancy*m*g; //Account for positive buoyancy bias
		}

		//Pitch
		if (isActive_Pitch)
		{
			ep_Pitch_prev = ep_Pitch;
			ep_Pitch = setPoint_Pitch - estimated_Pitch;
			ei_Pitch += ep_Pitch*dt;
			ed_Pitch = (ep_Pitch - ep_Pitch_prev)/dt;
			Ty = kp*ep_Pitch + ki*ei_Pitch + kd*ed_Pitch;
		}

		//Yaw
		if (isActive_Yaw)
		{
			ep_Yaw_prev = ep_Yaw;
			ep_Yaw = setPoint_Yaw - estimated_Yaw;
			ei_Yaw += ep_Yaw*dt;
			ed_Yaw = (ep_Yaw - ep_Yaw_prev)/dt;
			Tz = kp*ep_Yaw + ki*ei_Yaw + kd*ed_Yaw;
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
