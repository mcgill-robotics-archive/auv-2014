/**
 * @author Nick Speal
 * Test Node Publishes Inputs to Test Control Node
*/

#include "ros/ros.h"
#include "planner/setPoints.h"
#include "std_msgs/Float64.h"

int main(int argc, char **argv)
{
	ros::init(argc, argv, "testControls");
	ROS_DEBUG("Initialized testControls Node");
	ros::NodeHandle n;

	ros::Publisher setPointsPublisher = n.advertise<planner::setPoints>("setPoints", 1000);
	ros::Rate loop_rate(10);


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

	const int8_t zero = 0;
	while (ros::ok())
	{
		// process!
		//define msg

		planner::setPoints msg;


		//parameters

		
		//TODO figure this out - doesnt compile
			//Theres a problem with the 8 bit int data type, or something to do with that.

	    //n.param<int8_t>("setPoints/XPos/isActive", isActive_XPos, zero);
	    // n.param<int8_t>("setPoints/XSpeed/isActive", isActive_XSpeed, 0);

	    // n.param<int8_t>("setPoints/YPos/isActive", isActive_YPos, 0);
	    // n.param<int8_t>("setPoints/YSpeed/isActive", isActive_YSpeed, 0);

	    // n.param<int8_t>("setPoints/Depth/isActive", isActive_Depth, 0);

	    // n.param<int8_t>("setPoints/Yaw/isActive", isActive_Yaw, 0);
	    // n.param<int8_t>("setPoints/YawSpeed/isActive", isActive_YawSpeed, 0);

	    // n.param<int8_t>("setPoints/Pitch/isActive", isActive_YawSpeed, 0);


		n.param<double>("setPoints/XPos/value", setPoint_XPos, 0.0);
	    n.param<double>("setPoints/XSpeed/value", setPoint_XSpeed, 0.0);

	    n.param<double>("setPoints/YPos/value", setPoint_YPos, 0.0);
	    n.param<double>("setPoints/YSpeed/value", setPoint_YSpeed, 0.0);

	    n.param<double>("setPoints/Depth/value", setPoint_Depth, 0.0);

	    n.param<double>("setPoints/Yaw/value", setPoint_Yaw, 0.0);
	    n.param<double>("setPoints/YawSpeed/value", setPoint_YawSpeed, 0.0);

	    n.param<double>("setPoints/Pitch/value", setPoint_Pitch, 0.0);


		msg.XPos.data = setPoint_XPos;
		msg.XSpeed.data = setPoint_XSpeed;

		msg.YPos.data = setPoint_YPos;
		msg.YSpeed.data = setPoint_YSpeed;

		msg.Depth.data = setPoint_Depth;

		msg.Yaw.data = setPoint_Yaw;
		msg.YawSpeed.data = setPoint_YawSpeed;

		msg.Pitch.data = setPoint_Pitch;

		//isactive parameters doesnt work yet so these are the parameters 1/28
		msg.XPos.isActive = 1;
		msg.YPos.isActive = 0;
		msg.Depth.isActive = 0;
		msg.Yaw.isActive = 0;
		msg.Pitch.isActive = 0;
		msg.XSpeed.isActive = 0;
		msg.YSpeed.isActive = 0;
		msg.YawSpeed.isActive = 0;

		msg.Frame = "/target/gate";

		//parameters

		//n.param<std::float>("testParam", test_param, 99);
		//ROS_DEBUG("printing...");
		setPointsPublisher.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
