/**
 * @author Nick Speal
 * Test Node Publishes Inputs to Test Control Node
*/

#include "ros/ros.h"
#include "planner/setPoints.h"
int main(int argc, char **argv)
{
	ros::init(argc, argv, "testControls");
	ros::NodeHandle n;

	ros::Publisher setPointsPublisher = n.advertise<planner::setPoints>("setPoints", 1000);
	ros::Rate loop_rate(10);

	while (ros::ok())
	{
		// process!
		//define msg

		planner::setPoints msg;

		msg.XPos.data = 2.3;
		msg.XSpeed.data = 0;
		msg.YPos.data = 3.5;
		msg.Depth.data = 1.0;
		msg.Yaw.data = 0;
		msg.Pitch.data = 0;

		msg.XPos.isActive = 1;
		msg.YPos.isActive = 1;
		msg.Depth.isActive = 1;
		msg.Yaw.isActive = 1;
		msg.Pitch.isActive = 1;
		msg.XSpeed.isActive = 0;
		msg.YSpeed.isActive = 0;
		msg.YawSpeed.isActive = 0;
		//skip depth

		//parameters

		nh.param<std::float>("testParam", test_param, 99);
		ROS_INFO(test_param)

		setPointsPublisher.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}