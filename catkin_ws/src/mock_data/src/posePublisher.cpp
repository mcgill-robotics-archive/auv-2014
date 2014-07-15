#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include <ctime>
#include <sstream>

float randomFloat() {
  return (float)rand()/(float)RAND_MAX;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "posePublisher");
  ros::NodeHandle n;
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("poseStamped", 1000);
  ros::Rate loop_rate(10);

  srand((unsigned)time(0));

  while (ros::ok())
  {
    geometry_msgs::PoseStamped pose;
    pose.pose.orientation.x = randomFloat();
    pose.pose.orientation.y = randomFloat();
    pose.pose.orientation.z = randomFloat();
    pose.pose.orientation.w = randomFloat();

    std::stringstream ss;
    ss << "x, y, z, w: " << pose.pose.orientation.x << ", " << pose.pose.orientation.y << ", "<< pose.pose.orientation.z << ", "<< pose.pose.orientation.w;
   
    ROS_INFO("%s", ss.str().c_str());

    pose_pub.publish(pose);

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}

