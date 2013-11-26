#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depth");

  ros::NodeHandle n;

  ros::Publisher depth_pub = n.advertise<std_msgs::Float64>("depth", 1000);

  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    std_msgs::Float64 msg;
    msg.data = 1.0;

    ROS_INFO("%f", msg.data);

    depth_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
