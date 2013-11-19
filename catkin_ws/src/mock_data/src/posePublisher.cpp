#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Pose.h"
#include <ctime>
#include <sstream>

float randomFloat() {
  return (float)rand()/(float)RAND_MAX;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "posePublisher");
  ros::NodeHandle n;
  ros::Publisher pose_pub = n.advertise<geometry_msgs::Pose>("pose", 1000);
  ros::Rate loop_rate(10);

  srand((unsigned)time(0));

  while (ros::ok())
  {
    geometry_msgs::Pose pose;
    pose.orientation.x = randomFloat();
    pose.orientation.y = randomFloat();
    pose.orientation.z = randomFloat();
    pose.orientation.w = randomFloat();

    std::stringstream ss;
    ss << "x, y, z, w: " << pose.orientation.x << ", " << pose.orientation.y << ", "<< pose.orientation.z << ", "<< pose.orientation.w;
   
    ROS_INFO("%s", ss.str().c_str());

    pose_pub.publish(pose);

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}

