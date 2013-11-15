#include "ROS.h"

double Depth;

ROS::ROS()
{
	/**********************************************************************
	Constructor:
	
	Subscribes to all topics 
	
	The topic names and types are ready in our folder 
	~/RoboSub2014/Engineering/4. Sensor+State Estimation/Data_Format.docx.
	
	Basically, cach all info you get from the topics
	listed in the doc in private variables
	
	Make nice getter functions

	You will need to create 2 ROS nodes to test this:
		1. A node with the ROS object and a ROS_Tester object that has a main function and initializes and prints the output of the ROS object
		2. A Testing node that creates the topics and sends mock data. For example, depth = 1.0 1.5 2.0 and then starts again. The tester object (in the other node) will be printing all of this in the command line
	**********************************************************************/

}

void setDepth(const std_msgs::Float64::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f]", msg->data);
  Depth = msg->data;
}

double ROS::GetDepth()
{
    return Depth;
}

double ROS::GetX()
{
    return 0.0;
}

double ROS::GetY()
{
    return 0.0;
}

PoseObj ROS::GetPose()
{
}


/*
Vector ROS::Velocity()
{
}
*/
double ROS::GetPressure()
{
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ROS");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("depth", 1000, setDepth);

  ros::spin();

  return 0;
}
