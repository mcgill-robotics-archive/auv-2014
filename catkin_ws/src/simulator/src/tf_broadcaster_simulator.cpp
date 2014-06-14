/**
* Subscribes to pose of each important object in the simulator and broadcasts a transform between it and the simulator world frame
* @author Nick Speal
* @author Dwijesh Bhageerutty
*/




#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/LinkStates.h"
#include "boost/bind.hpp"

float PI = 3.1415926535897932384626;

void poseCallback0(const gazebo_msgs::ModelStates msg){
  /*
  * Creates the transform between the robot and the world
  */

  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg.pose[0].position.x, msg.pose[0].position.y, msg.pose[0].position.z) );
  transform.setRotation(tf::Quaternion(msg.pose[0].orientation.x, msg.pose[0].orientation.y, msg.pose[0].orientation.z, msg.pose[0].orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "world", "robot_SO"));


  //rotate frame of robot to agree with McGill Robotics Convention (x forward)
  static tf::TransformBroadcaster br2;
  static tf::Transform transform2;

  static tfScalar pitch = 0; //-rotate about the camera y?
  static tfScalar roll = PI;  //pi //rotate about camera x? 
  static tfScalar yaw = PI/2; //rotate about camera z? but I dont really understand this....
  static tf::Quaternion quatInstance;
  quatInstance.tf::Quaternion::setEuler(pitch, roll, yaw);

  //transform2.setOrigin( tf::Vector3(px,py,pz) );
  transform2.setRotation(quatInstance);
  br2.sendTransform(tf::StampedTransform(transform2, ros::Time(0), "robot_SO", "robot_reoriented"));
}

void poseCallback1(const gazebo_msgs::LinkStates msg){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg.pose[0].position.x, msg.pose[0].position.y, msg.pose[0].position.z) );
  transform.setRotation(tf::Quaternion(msg.pose[0].orientation.x, msg.pose[0].orientation.y, msg.pose[0].orientation.z, msg.pose[0].orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "world", "robotBody_SO"));
}

//sub2 deleted

/**
*Gate Subscriber
*/
void poseCallback3(const gazebo_msgs::ModelStates msg){
  //TF between world and gate_SO
  int n = 1; //position of the gate in the array of models  
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg.pose[n].position.x, msg.pose[n].position.y, msg.pose[n].position.z) );
  transform.setRotation(tf::Quaternion(msg.pose[n].orientation.x, msg.pose[n].orientation.y, msg.pose[n].orientation.z, msg.pose[n].orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "world", "gate_SO"));

  //TF between gate_SO and gate_center_sim -center of the gate as perceived in the simulator
  //position of the gate_center_sim origin as seen in the gate_SO frame
  static tf::TransformBroadcaster br2;
  static tf::Transform transform2;
  static tfScalar px = 1.5; //position
  static tfScalar py = -0.6;
  static tfScalar pz = 0;
  //orientation
  static tfScalar pitch = PI/2;  //-rotate about the world z, or gate y
  static tfScalar roll = PI/2;  //rotate about both x axes 
  static tfScalar yaw = 0; //rotate about world -y or gate +z-axis - but I dont really understand this....
  static tf::Quaternion quatInstance;
  quatInstance.tf::Quaternion::setEuler(pitch, roll, yaw);

  transform2.setOrigin( tf::Vector3(px,py,pz) );
  transform2.setRotation(quatInstance);
  br2.sendTransform(tf::StampedTransform(transform2, ros::Time(0), "gate_SO", "gate_center_sim"));


}


/**
*Line1 Subscriber
*/
void poseCallback4(const gazebo_msgs::ModelStates msg){
  int n = 2; //position of the line in the array of models
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg.pose[n].position.x, msg.pose[n].position.y, msg.pose[n].position.z) );
  transform.setRotation(tf::Quaternion(msg.pose[n].orientation.x, msg.pose[n].orientation.y, msg.pose[n].orientation.z, msg.pose[n].orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "world", "line1_SO"));
}

/**
*Camera 1 Subscriber
*/
void poseCallback5(const gazebo_msgs::LinkStates msg){
  //tf between camera 1 and the simulator world origin
  int n = 2; //position of the canera in the array of links
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg.pose[n].position.x, msg.pose[n].position.y, msg.pose[n].position.z) );
  transform.setRotation(tf::Quaternion(msg.pose[n].orientation.x, msg.pose[n].orientation.y, msg.pose[n].orientation.z, msg.pose[n].orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "world", "camera1_SO"));

  //rotate frame of camera to aggree with McGill Robotics Convention (x forward)
  static tf::TransformBroadcaster br2;
  static tf::Transform transform2;

  static tfScalar pitch = 0; //-rotate about the camera y?
  static tfScalar roll = PI;  //pi //rotate about camera x? 
  static tfScalar yaw = 0; //rotate about camera z? but I dont really understand this....
  static tf::Quaternion quatInstance;
  quatInstance.tf::Quaternion::setEuler(pitch, roll, yaw);

  //transform2.setOrigin( tf::Vector3(px,py,pz) );
  transform2.setRotation(quatInstance);
  br2.sendTransform(tf::StampedTransform(transform2, ros::Time(0), "camera1_SO", "camera1_reoriented"));

}

/**
*Camera 2 Subscriber
*/
void poseCallback6(const gazebo_msgs::LinkStates msg){
  //tf between camera 2 and the simulator world origin
  int n = 3; //position of the canera in the array of links
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg.pose[n].position.x, msg.pose[n].position.y, msg.pose[n].position.z) );
  transform.setRotation(tf::Quaternion(msg.pose[n].orientation.x, msg.pose[n].orientation.y, msg.pose[n].orientation.z, msg.pose[n].orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "world", "camera2_SO"));

  //rotate frame of camera to aggree with McGill Robotics Convention (x forward)
  static tf::TransformBroadcaster br2;
  static tf::Transform transform2;

  static tfScalar pitch = 0; //-rotate about the camera y?
  static tfScalar roll = PI;  //pi //rotate about camera x? 
  static tfScalar yaw = 0; //rotate about camera z? but I dont really understand this....
  static tf::Quaternion quatInstance;
  quatInstance.tf::Quaternion::setEuler(pitch, roll, yaw);

  //transform2.setOrigin( tf::Vector3(px,py,pz) );
  transform2.setRotation(quatInstance);
  br2.sendTransform(tf::StampedTransform(transform2, ros::Time(0), "camera2_SO", "camera2_reoriented"));
}

/**
*Camera 3 Subscriber
*/
void poseCallback7(const gazebo_msgs::LinkStates msg){
  //tf between camera 1 and the simulator world origin
  int n = 4; //position of the canera in the array of links
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg.pose[n].position.x, msg.pose[n].position.y, msg.pose[n].position.z) );
  transform.setRotation(tf::Quaternion(msg.pose[n].orientation.x, msg.pose[n].orientation.y, msg.pose[n].orientation.z, msg.pose[n].orientation.w));
  br.sendTransform(tf::StampedTransform(transform, ros::Time(0), "world", "camera3_SO"));

  //rotate frame of camera to aggree with McGill Robotics Convention (x forward)
  static tf::TransformBroadcaster br2;
  static tf::Transform transform2;

  static tfScalar pitch = PI/2; //-rotate about the camera y?
  static tfScalar roll = 0;  //pi //rotate about camera x? 
  static tfScalar yaw = -PI/2; //rotate about camera z? but I dont really understand this....
  static tf::Quaternion quatInstance;
  quatInstance.tf::Quaternion::setEuler(pitch, roll, yaw);

  //transform2.setOrigin( tf::Vector3(px,py,pz) );
  transform2.setRotation(quatInstance);
  br2.sendTransform(tf::StampedTransform(transform2, ros::Time(0), "camera3_SO", "camera3_reoriented"));


}



int main(int argc, char** argv){
  ROS_INFO("tf_broadcaster_simulator initialized");
  ros::init(argc, argv, "tf_broadcaster_simulator");
  ros::NodeHandle node;
  ros::Subscriber sub0 = node.subscribe("/gazebo/model_states/", 10, &poseCallback0); //robot
  ros::Subscriber sub1 = node.subscribe("/gazebo/link_states/", 10, &poseCallback1); //robotbody (same as robot, except a link not a body. Initially at least...seems to stay coincident)
  //sub2 deleted
  ros::Subscriber sub3 = node.subscribe("/gazebo/model_states/", 10, &poseCallback3); //gate
  ros::Subscriber sub4 = node.subscribe("/gazebo/model_states/", 10, &poseCallback4); //line1
  ros::Subscriber sub5 = node.subscribe("/gazebo/link_states/", 10, &poseCallback5); //cam1_SO
  ros::Subscriber sub6 = node.subscribe("/gazebo/link_states/", 10, &poseCallback6); //cam2_SO
  ros::Subscriber sub7 = node.subscribe("/gazebo/link_states/", 10, &poseCallback7); //cam3_SO

  ros::Rate loop_rate(30);
  ros::spin();
  return 0;
};

