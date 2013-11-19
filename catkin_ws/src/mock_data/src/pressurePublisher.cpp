#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <ctime>
#include <sstream>

float randomFloat() {
  return (float)rand()/(float)RAND_MAX;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pressurePublisher");
  ros::NodeHandle n;
  ros::Publisher pressure_pub = n.advertise<std_msgs::Float32>("pressure", 1000);
  ros::Rate loop_rate(10);

  srand((unsigned)time(0));

  while (ros::ok())
  {
    std_msgs::Float32 pressure;
    pressure.data = randomFloat();

    std::stringstream ss;
    ss << "pressure: " <<  pressure.data;

    ROS_INFO("%s", ss.str().c_str());

    pressure_pub.publish(pressure);

    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}

