#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <ctime>
#include <sstream>

float randomFloat() {
  return (float)rand()/(float)RAND_MAX;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "depthPublisher");
  ros::NodeHandle n;
  ros::Publisher depth_pub = n.advertise<std_msgs::Float32>("depth", 1000);
  ros::Rate loop_rate(10);

  srand((unsigned)time(0));

  while (ros::ok())
  {
    std_msgs::Float32 depth;
    depth.data = randomFloat();

    std::stringstream ss;
    ss << "depth: " <<  depth.data;

    ROS_INFO("%s", ss.str().c_str());

    depth_pub.publish(depth);

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}

