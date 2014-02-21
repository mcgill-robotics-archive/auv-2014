#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/LinkStates.h"


void poseCallback0(const gazebo_msgs::ModelStates msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg.pose[0].position.x, msg.pose[0].position.y, msg.pose[0].position.z) );
  transform.setRotation(tf::Quaternion(msg.pose[0].orientation.x, msg.pose[0].orientation.y, msg.pose[0].orientation.z, msg.pose[0].orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "world", "robot"));
}

void poseCallback1(const gazebo_msgs::LinkStates msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg.pose[0].position.x, msg.pose[0].position.y, msg.pose[0].position.z) );
  transform.setRotation(tf::Quaternion(msg.pose[0].orientation.x, msg.pose[0].orientation.y, msg.pose[0].orientation.z, msg.pose[0].orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "world", "robotBody"));
}

void poseCallback2(const gazebo_msgs::LinkStates msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg.pose[3].position.x, msg.pose[3].position.y, msg.pose[3].position.z) );
  transform.setRotation(tf::Quaternion(msg.pose[3].orientation.x, msg.pose[3].orientation.y, msg.pose[3].orientation.z, msg.pose[3].orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "world", "robotcam3"));
}

/**
*Gate Subscriber
*/
void poseCallback3(const gazebo_msgs::ModelStates msg){
  int n = 1; //position of the gate in the array of models
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg.pose[n].position.x, msg.pose[n].position.y, msg.pose[n].position.z) );
  transform.setRotation(tf::Quaternion(msg.pose[n].orientation.x, msg.pose[n].orientation.y, msg.pose[n].orientation.z, msg.pose[n].orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "world", "gate"));
}ros

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster_simulator");
  ros::NodeHandle node;
  ros::Subscriber sub0 = node.subscribe("/gazebo/model_states/", 10, &poseCallback0); //robot
  ros::Subscriber sub1 = node.subscribe("/gazebo/link_states/", 10, &poseCallback1); //robotbody (same as robot, except a link not a body. Initially at least...)
  ros::Subscriber sub2 = node.subscribe("/gazebo/link_states/", 10, &poseCallback2); //robotcam3
  ros::Subscriber sub3 = node.subscribe("/gazebo/model_states/", 10, &poseCallback3); //gate
  

  ros::spin();
  return 0;
};

