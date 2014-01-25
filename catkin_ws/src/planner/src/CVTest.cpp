#include "CVTest.h"
double X; double Y; double Z;
double Yaw; double Pitch;

void setGazeboPretendThings(gazebo_msgs::ModelStates msg) {
  X = msg.pose[1].position.x;
  Y = msg.pose[1].position.y;
  Z = msg.pose[1].position.z;
  Yaw = 0;
  Pitch = 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "CVTest");

  ros::NodeHandle n;

  ros::Subscriber CV_sub = n.subscribe("gazebo/model_states", 1000, setGazeboPretendThings);

  ros::Publisher pose_pub = n.advertise<computer_vision::VisibleObjectData>("visible_data", 1000);

  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {
    computer_vision::VisibleObjectData msgCV;
    msgCV.x_distance = X;
    msgCV.y_distance = Y;
    msgCV.z_distance = Z;
    msgCV.yaw_angle = Yaw;
    msgCV.pitch_angle = Pitch;

    pose_pub.publish(msgCV);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
