#include "rosTest.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dummy_sensors");

  ros::NodeHandle n;

  ros::Publisher depth_pub = n.advertise<std_msgs::Float64>("depth_data", 1000);

  ros::Publisher pressure_pub = n.advertise<std_msgs::Float64>("pressure_data", 1000);

  ros::Publisher CV_pub = n.advertise<geometry_msgs::Twist>("CV", 1000);

  ros::Publisher IMU_pub = n.advertise<geometry_msgs::Quaternion>("imu_data", 1000);

  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    geometry_msgs::Twist msgCV;
    msgCV.linear.x = 1.0;
    msgCV.linear.y = 1.0;
    msgCV.linear.z = 1.0;
    msgCV.angular.x = 1.0;
    msgCV.angular.y = 1.0;
    msgCV.angular.z = 1.0;

    geometry_msgs::Quaternion msgIMU;
    msgIMU.x = 4.0;
    msgIMU.y = 5.0;
    msgIMU.z = 6.0;
    msgIMU.w = 7.0;

    std_msgs::Float64 msgD;
    msgD.data = 1.0;

    std_msgs::Float64 msgP;
    msgP.data = 0.1;
/*
    ROS_INFO("%f,%f -- %f,%f\n", msgD.data,msgD.raw, msgP.data, msgP.raw);
    ROS_INFO("%f,%f,%f -- %f,%f,%f", msgCV.angular.x,msgCV.angular.y,msgCV.angular.z,msgCV.linear.x, msgCV.linear.y, msgCV.linear.z);
*/
    depth_pub.publish(msgD);
    pressure_pub.publish(msgP);
    CV_pub.publish(msgCV);
    IMU_pub.publish(msgIMU);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
