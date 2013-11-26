#include "rosTest.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sensors");

  ros::NodeHandle n;

  ros::Publisher depth_pub = n.advertise<sprint1::Depth>("depth", 1000);

  ros::Publisher pressure_pub = n.advertise<sprint1::Pressure>("pressure", 1000);

  ros::Publisher CV_pub = n.advertise<geometry_msgs::Twist>("CV", 1000);

  ros::Publisher DVL_pub = n.advertise<geometry_msgs::Twist>("DVL", 1000);

  ros::Publisher IMU_pub = n.advertise<geometry_msgs::Pose>("IMU", 1000);

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

    geometry_msgs::Twist msgDVL;
    msgDVL.linear.x = 2.0;
    msgDVL.linear.y = 2.0;
    msgDVL.linear.z = 2.0;
    msgDVL.angular.x = 2.0;
    msgDVL.angular.y = 2.0;
    msgDVL.angular.z = 2.0;

    geometry_msgs::Pose msgIMU;
    msgIMU.position.x = 1.0;
    msgIMU.position.y = 2.0;
    msgIMU.position.z = 3.0;
    msgIMU.orientation.x = 4.0;
    msgIMU.orientation.y = 5.0;
    msgIMU.orientation.z = 6.0;
    msgIMU.orientation.w = 7.0;

    sprint1::Depth msgD;
    msgD.data = 1.0;
    msgD.raw = 1.23;

    sprint1::Pressure msgP;
    msgP.data = 0.1;
    msgP.raw = 0.123;

    ROS_INFO("%f,%f -- %f,%f\n", msgD.data,msgD.raw, msgP.data, msgP.raw);
    ROS_INFO("%f,%f,%f -- %f,%f,%f", msgCV.angular.x,msgCV.angular.y,msgCV.angular.z,msgCV.linear.x, msgCV.linear.y, msgCV.linear.z);

    depth_pub.publish(msgD);
    pressure_pub.publish(msgP);
    CV_pub.publish(msgCV);
    DVL_pub.publish(msgDVL);
    IMU_pub.publish(msgIMU);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
