#include "ros/ros.h"
#include "std_msgs/String.h"
#include "robosub_msg/CurrentCVTask.h"

/**

New node to publish hard coded strings to mimick commands coming from the planner
Used to unit test other code (controls, etc..)

*/


int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "FakeTaskPublisher");

  ros::NodeHandle n;

  ros::Publisher taskPubFront = n.advertise<robosub_msg::CurrentCVTask>("currentCVTask_Front", 1000);
  ros::Publisher taskPubDown = n.advertise<robosub_msg::CurrentCVTask>("currentCVTask_Down", 1000);


  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    robosub_msg::CurrentCVTask msgFront;
    robosub_msg::CurrentCVTask msgDown;

    msgFront.currentCVTask = msgFront.GATE;
    msgDown.currentCVTask = msgDown.NOTHING;

    taskPubFront.publish(msgFront);
    taskPubDown.publish(msgDown);


    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
