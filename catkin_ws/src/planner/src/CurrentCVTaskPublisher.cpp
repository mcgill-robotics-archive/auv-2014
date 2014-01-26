#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

/**

New node to publish hard coded strings to mimick commands coming from the planner
Used to unit test other code (controls, etc..)

*/


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "FakeTaskPublisher");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  ros::Publisher taskPubFront = n.advertise<std_msgs::String>("currentCVTask_Front", 1000);
  ros::Publisher taskPubDown = n.advertise<std_msgs::String>("currentCVTask_Down", 1000);


  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    std_msgs::String msgFront;
    std_msgs::String msgDown;

    msgFront.data = "gate";
    msgDown.data = "";

    taskPubFront.publish(msgFront);
    taskPubDown.publish(msgDown);


    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}