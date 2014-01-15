/**
 * @author Nick Speal
 * Test Node Publishes Inputs to Test Control Node
*/

#include "ros/ros.h"
#include "planer/setPoints.h"
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

		const planner::setPoints msg

		msg.Xpos.data = 0;
		msg.Ypos.data = 0;
		msg.Depth.data = -3;
		msg.Yaw.data = 0;
		msg.Pitch.data = 0;

		msg.Xpos.isActive = 1;
		msg.Ypos.isActive = 1;
		msg.Depth.isActive = 1;
		msg.Yaw.isActive = 1;
		msg.Pitch.isActive = 1;
		msg.XSpeed.isActive = 0;
		msg.YSpeed.isActive = 0;
		msg.YawSpeed.isActive = 0;
		//skip depth

		setPointsPublisher.publish(msg);
		ros::spinOnce();
		loop_rate.sleep()
	}
	return 0
}