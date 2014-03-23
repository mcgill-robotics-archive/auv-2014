
/*

First integrated 5DOF control system
Subscribes to setpoints and estimated states and publishes a net wrench to minimize error.
Open Loop Speed Control.	

Created by Nick Speal Jan 10.
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
remove old depthcontroller relics

subscribe to estimated position from planner?
different gains for different axes
add a saturate functionality to the forces
	should integral error accumulate?
fix coordinate system integration with Gen and Mathieu




/*
Simulator needs
-----------------

drag
apply force in body frame

*/



/*
Roadmap



*/
#include "ros/ros.h"
#include <ros/console.h> //to change verbosity of ROSINFO ROS_DEBUG etc

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"
#include "std_msgs/Float64.h"
#include "planner/setPoints.h"	
#include "computer_vision/VisibleObjectData.h"
#include "controls/DebugControls.h"	

//global vars
double z_des = 0;
double z_est = 0;

double setPoint_XPos = 0; 
double setPoint_YPos = 0;
double setPoint_Depth = 0;
double setPoint_Yaw = 0;
double setPoint_Pitch = 0;
double setPoint_XSpeed = 0;
double setPoint_YSpeed = 0;
double setPoint_YawSpeed = 0;

int8_t isActive_XPos = 0;	
int8_t isActive_YPos = 0;
int8_t isActive_Depth = 0;
int8_t isActive_Yaw = 0;
int8_t isActive_Pitch = 0;
int8_t isActive_XSpeed = 0;
int8_t isActive_YSpeed = 0;
int8_t isActive_YawSpeed = 0;

double estimated_XPos = 0;
double estimated_YPos = 0;
double depth = 0;
double estimated_Pitch = 0;
double estimated_Yaw = 0;

double F_MAX;
double T_MAX;

double EP_XPOS_MAX;
double EP_YPOS_MAX;
double XSPEED_MAX;
double YSPEED_MAX;

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

void estimatedState_callback(const computer_vision::VisibleObjectData data)

{
	ROS_DEBUG("Subscriber received estimated data");
    
    estimated_XPos = data.x_distance;
    estimated_YPos = data.y_distance;
    estimated_Pitch = data.pitch_angle; 
	estimated_Yaw = data.yaw_angle; 
}

void depth_callback(const std_msgs::Float64 data)
{
	depth = data.data;
}

float output_limit_check(float value, float min, float max, char* value_name ){
	//returns zero and warns if out of range
	if (value > max | value < min) { //out of range
		ROS_WARN("%s: value has been exceeded. Value is %f. Setting to 0.", value_name, value);
		value = 0;
		return value;
	}
	else {
		return value;
	}
}

float saturate(float value, float max, char* value_name) {
	//saturates if out of range
	if (value > max) {
		ROS_INFO("%s: value has been exceeded. Value was %f but has been set to %f", value_name, value, max);
		value=max;
	}
	else if (value < -1*max)
	{
		ROS_INFO("%s: value has been exceeded. Value was %f but has been set to %f", value_name, value, -1*max);
		value=-1*max;
	}

	return value;
}

int main(int argc, char **argv)
{
	//Create ROS Node
	ros::init(argc,argv,"controls");
	ros::NodeHandle n;

	//specify ROSCONSOLE verbosity level
	//Fred Lafrance: temporarily commented because of compile error	
	//if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
	//{
   	//	ros::console::notifyLoggerLevelsChanged();
	//}

	//Parameters
	double m; //mass in kg
	double g;
	double cd; // drag coefficient
	double buoyancy; // %percent buoyancy
	double dt; //temporary! TODO update this dynamically

	//Gains for the Proportional, Integral, and Derivative controllers
	double kp;
    double ki;
    double kd;

    double kp_xPos;
    double ki_xPos;
    double kd_xPos;

    double kp_yPos;
    double ki_yPos;
    double kd_yPos;

    double kp_Depth;
    double ki_Depth;
    double kd_Depth;

    double kp_Yaw;
    double ki_Yaw;
    double kd_Yaw;

    double kp_Pitch;
    double ki_Pitch;
    double kd_Pitch;


    //ROS Params

    n.param<double>("gains/kp", kp, 0.0);
    n.param<double>("gains/ki", ki, 0.0);
    n.param<double>("gains/kd", kd, 0.0);

    n.param<double>("gains/kp_xPos", kp_xPos, 0.0);
    n.param<double>("gains/ki_xPos", ki_xPos, 0.0);
    n.param<double>("gains/kd_xPos", kd_xPos, 0.0);

    n.param<double>("gains/kp_yPos", kp_yPos, 0.0);
    n.param<double>("gains/ki_yPos", ki_yPos, 0.0);
    n.param<double>("gains/kd_yPos", kd_yPos, 0.0);
    
    n.param<double>("gains/kp_Depth", kp_Depth, 0.0);
    n.param<double>("gains/ki_Depth", ki_Depth, 0.0);
    n.param<double>("gains/kd_Depth", kd_Depth, 0.0);

    n.param<double>("gains/kp_Pitch", kp_Pitch, 0.0);
    n.param<double>("gains/ki_Pitch", ki_Pitch, 0.0);
    n.param<double>("gains/kd_Pitch", kd_Pitch, 0.0);

    n.param<double>("gains/kp_Yaw", kp_Yaw, 0.0);
    n.param<double>("gains/ki_Yaw", ki_Yaw, 0.0);
    n.param<double>("gains/kd_Yaw", kd_Yaw, 0.0);

   
    n.param<double>("coefs/mass", m, 30.0);
    n.param<double>("coefs/buoyancy", buoyancy, 0.02);
    n.param<double>("coefs/drag", cd, 0.0);
    n.param<double>("coefs/gravity", g, 9.81);
    n.param<double>("coefs/dt", dt, 0.01);

	n.param<double>("force/max", F_MAX, 0.0);
	n.param<double>("torque/max", T_MAX, 0.0);

	n.param<double>("ep_XPos/max", EP_XPOS_MAX, 0.0);
	n.param<double>("ep_YPos/max", EP_YPOS_MAX, 0.0);
	n.param<double>("XSpeed/max", XSPEED_MAX, 0.0);
	n.param<double>("YSpeed/max", YSPEED_MAX, 0.0);


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
	ros::Subscriber estimatedState_subscriber = n.subscribe("/front_cv_data", 1000, estimatedState_callback);
	ros::Subscriber depth_subscriber = n.subscribe("/stateEstimation/depth", 1000, depth_callback);
	//TO DO: add clock subscription

	//ROS Publisher setup
	ros::Publisher wrench_publisher = n.advertise<geometry_msgs::Wrench>("/controls/wrench", 100);
	geometry_msgs::Wrench wrenchMsg; //define variable to publish
	ros::Publisher debug_publisher = n.advertise<controls::DebugControls>("/controls/debug", 100); // debug publisher with custom debug msg
	controls::DebugControls debugMsg;

	ros::Rate loop_rate(1/dt); 
	
	bool setPointsIsPublished = 0;
	bool estimatedStateIsPublished = 0;

	ROS_INFO("controls node waiting for setPoints to be published...");
	while (setPointsIsPublished == 0)
	{
		ROS_DEBUG_THROTTLE(2,"Waiting...");
		setPointsIsPublished = 1;		 
		if (setPoints_subscriber.getNumPublishers() == 0) {setPointsIsPublished = 0;}
	}

	ROS_INFO("All Subscribers Live. Starting Controller!");
	while(ros::ok())
	{
		ros::spinOnce();	//Updates all variables 
		
		// Decoupled Controllers

		//zero unless otherwise specified
		Fx = 0;
    	Fy = 0;
    	Fz = 0;
    	Ty = 0;
    	Tz = 0;

    	//Check if estimated state is being published
    	if (estimatedState_subscriber.getNumPublishers() == 0) {estimatedStateIsPublished = 0;}
		else {estimatedStateIsPublished = 1;}
		//Check if depth is being published
		if (depth_subscriber.getNumPublishers() == 0) {depthIsPublished = 0;}
		else {depthIsPublished = 1;}
		

		//X 
		if (isActive_XPos)
		{
			if (estimatedStateIsPublished)
			{
				ep_XPos_prev = ep_XPos;
				ep_XPos = setPoint_XPos - estimated_XPos;
				ep_XPos=saturate(ep_XPos, EP_XPOS_MAX, "X Position Error term");
				ei_XPos += ep_XPos*dt;
				ed_XPos = (ep_XPos - ep_XPos_prev)/dt;
				Fx = kp_xPos*ep_XPos + ki_xPos*ei_XPos + kd_xPos*ed_XPos;
				Fx *= -1; //flip direction to account for relative coordinate system
				//ROS_INFO("controlling xpos");
				//ROS_INFO("proportional error: %f | Time: %f", ep_XPos, ros::Time::now().toSec());
			}
			else
			{
				ROS_WARN("X-Position Control is active, but estimated state is not published")
			}
		}
        
        if (isActive_XSpeed)
        {
        	OL_coef_x = 5;
           	setPoint_XSpeed=saturate(setPoint_XSpeed, XSPEED_MAX, "X Speed");
        	Fx = OL_coef_x*setPoint_XSpeed;
        	//ROS_INFO("controlling xspeed");
        }

        //Y 
		if (isActive_YPos)
		{
			if (estimatedStateIsPublished)
			{
				ep_YPos_prev = ep_YPos;
				ep_YPos = setPoint_YPos - estimated_YPos;
				ep_YPos=saturate(ep_YPos, EP_YPOS_MAX, "Y Position Error term");
				ei_YPos += ep_YPos*dt;
				ed_YPos = (ep_YPos - ep_YPos_prev)/dt;
				Fy = kp_yPos*ep_YPos + ki_yPos*ei_YPos + kd_yPos*ed_YPos;
				Fy *= -1; //flip direction to account for relative coordinate system TODO check with CV }
			}
			else
			{
				ROS_WARN("Y-Position Control is active, but estimated state is not published")
			}
        }
        if (isActive_YSpeed)
        {
        	OL_coef_y = 5;
        	setPoint_YSpeed=saturate(setPoint_YSpeed, YSPEED_MAX, "Y Speed");
        	Fy = OL_coef_y*setPoint_YSpeed;
        }

        //Z 
		if (isActive_Depth)

		{
			if (depthIsPublished)
			{
				//ROS_INFO("setPointDepth: %f | estimatedDepth: %f", setPoint_Depth, depth);
				ep_Depth_prev = ep_Depth;
				ep_Depth = setPoint_Depth - depth;
				ei_Depth += ep_Depth*dt;
				ed_Depth = (ep_Depth - ep_Depth_prev)/dt;
				Fz = kp_Depth*ep_Depth + ki_Depth*ei_Depth + kd_Depth*ed_Depth;
				//Fz *= -1; //flip direction to account for relative coordinate system - don't have to do this because depth isnt relative like the others
				//Fz = 0; // workaround temp
				//Fz += buoyancy*m*g; //Account for positive buoyancy bias
			}
			else
			{
				ROS_WARN("Depth position Control is active, but /depth is not published")
			}
		}

		//Pitch
		if (isActive_Pitch)
		{
			ep_Pitch_prev = ep_Pitch;
			ep_Pitch = setPoint_Pitch - estimated_Pitch;
			ei_Pitch += ep_Pitch*dt;
			ed_Pitch = (ep_Pitch - ep_Pitch_prev)/dt;
			Ty = kp_Pitch*ep_Pitch + ki_Pitch*ei_Pitch + kd_Pitch*ed_Pitch;
		}

		//Yaw
		if (isActive_Yaw)
		{
			ep_Yaw_prev = ep_Yaw;
			ep_Yaw = setPoint_Yaw - estimated_Yaw;
			ei_Yaw += ep_Yaw*dt;
			ed_Yaw = (ep_Yaw - ep_Yaw_prev)/dt;
			Tz = kp_Yaw*ep_Yaw + ki_Yaw*ei_Yaw + kd_Yaw*ed_Yaw;
		}

		if (isActive_YawSpeed)
        {
        	OL_coef_yaw = 1;
        	Tz = OL_coef_yaw*setPoint_YawSpeed;
        }
		//Limit check for output force/torque values
		Fx=saturate(Fx, F_MAX, "Force: X");
		Fy=saturate(Fy, F_MAX, "Force: Y");
		Fz=saturate(Fz, F_MAX, "Force: Z");
		Ty=saturate(Ty, T_MAX, "Torque: Y");
		Tz=saturate(Tz, T_MAX, "Torque: Z");

		// Assemble Wrench
		wrenchMsg.force.x = Fx;
		wrenchMsg.force.y = Fy;
		wrenchMsg.force.z = Fz;
		wrenchMsg.torque.x = 0;	//no active roll control
		wrenchMsg.torque.y = Ty;
		wrenchMsg.torque.z = Tz;

		// Assemble Debug

			// Error
			debugMsg.xError.proportional = ep_XPos;
			debugMsg.yError.proportional = ep_YPos;
			debugMsg.depthError.proportional = ep_Depth;
			debugMsg.pitchError.proportional = ep_Pitch;
			debugMsg.yawError.proportional = ep_Yaw;

			debugMsg.xError.integral = ei_XPos;
			debugMsg.yError.integral = ei_YPos;
			debugMsg.depthError.integral = ei_Depth;
			debugMsg.pitchError.integral = ei_Pitch;
			debugMsg.yawError.integral = ei_Yaw;

			debugMsg.xError.derivative = ed_XPos;
			debugMsg.yError.derivative = ed_YPos;
			debugMsg.depthError.derivative = ed_Depth;
			debugMsg.pitchError.derivative = ed_Pitch;
			debugMsg.yawError.derivative = ed_Yaw;

			// Gains
			
			debugMsg.xGain.proportional = kp;
			debugMsg.xGain.integral = ki;
			debugMsg.xGain.derivative = kd;

			/*
			debugMsg.xGain.proportional = kp_xPos;
			debugMsg.yGain.proportional = kp_yPos;
			debugMsg.depthGain.proportional = kp_Depth;
			debugMsg.pitchGain.proportional = kp_Pitch;
			debugMsg.yawGain.proportional = kp_Yaw;

			debugMsg.xGain.integral = ki_xPos;
			debugMsg.yGain.integral = ki_yPos;
			debugMsg.depthGain.integral = ki_Depth;
			debugMsg.pitchGain.integral = ki_Pitch;
			debugMsg.yawGain.integral = ki_Yaw;

			debugMsg.xGain.derivative = kd_xPos;
			debugMsg.yGain.derivative = kd_yPos;
			debugMsg.depthGain.derivative = kd_Depth;
			debugMsg.pitchGain.derivative = kd_Pitch;
			debugMsg.yawGain.derivative = kd_Yaw;
			*/
			
			// Forces
			debugMsg.xForce.proportional = kp*ep_XPos;
			debugMsg.yForce.proportional = kp*ep_YPos;
			debugMsg.depthForce.proportional = kp*ep_Depth;
			debugMsg.pitchForce.proportional = kp*ep_Pitch;
			debugMsg.yawForce.proportional = kp*ep_Yaw;

			debugMsg.xForce.integral = kp*ei_XPos;
			debugMsg.yForce.integral = kp*ei_YPos;
			debugMsg.depthForce.integral = kp*ei_Depth;
			debugMsg.pitchForce.integral = kp*ei_Pitch;
			debugMsg.yawForce.integral = kp*ei_Yaw;

			debugMsg.xForce.derivative = kp*ed_XPos;
			debugMsg.yForce.derivative = kp*ed_YPos;
			debugMsg.depthForce.derivative = kp*ed_Depth;
			debugMsg.pitchForce.derivative = kp*ed_Pitch;
			debugMsg.yawForce.derivative = kp*ed_Yaw;


		wrench_publisher.publish(wrenchMsg);
		debug_publisher.publish(debugMsg);
		loop_rate.sleep();


	}
	return 0;
}
