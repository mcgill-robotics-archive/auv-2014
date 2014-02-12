#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/LinkStates.h"

void poseCallback(const gazebo_msgs::ModelStates msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg.pose[0].position.x, msg.pose[0].position.y, msg.pose[0].position.z) );
  transform.setRotation(tf::Quaternion(msg.pose[0].orientation.x, msg.pose[0].orientation.y, msg.pose[0].orientation.z, msg.pose[0].orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robot"));
}

void poseCallback1(const gazebo_msgs::LinkStates msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg.pose[0].position.x, msg.pose[0].position.y, msg.pose[0].position.z) );
  transform.setRotation(tf::Quaternion(msg.pose[0].orientation.x, msg.pose[0].orientation.y, msg.pose[0].orientation.z, msg.pose[0].orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robotBody"));
}

void poseCallback2(const gazebo_msgs::LinkStates msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg.pose[3].position.x, msg.pose[3].position.y, msg.pose[3].position.z) );
  transform.setRotation(tf::Quaternion(msg.pose[3].orientation.x, msg.pose[3].orientation.y, msg.pose[3].orientation.z, msg.pose[3].orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "robotcam3"));
}

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster_simulator");
  ros::NodeHandle node;
  ros::Subscriber sub1 = node.subscribe("/gazebo/model_states/", 10, &poseCallback);
  ros::Subscriber sub2 = node.subscribe("/gazebo/link_states/", 10, &poseCallback1);
  ros::Subscriber sub3 = node.subscribe("/gazebo/link_states/", 10, &poseCallback2);
  
  ros::spin();
  return 0;
};

