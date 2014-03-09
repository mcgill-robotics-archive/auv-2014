/**
* Listens to TF between robot and objects and publishes fake CV data
* used in simulation to replace CV for unit tests or to validate CV
*@author Nick Speal
*@modified March 9
*/

#include "distanceCalculator.h"

int CV_PUBLISHING_RATE = 2; //publish at 2 Hz, just like CV
planner::currentCVTask currentCVTask_Front;

/* Generic current task callback

    There's gotta be a way to only use one callback for both the front and bottom current CV tasks. But I can't figure it out, so I'll leave it here for later.

    void currentCVTask_callback(const ros::MessageEvent<planner::currentCVTask const>& event)
    {
      const ros::M_string& header = event.getConnectionHeader();
      std::string topic = header.at("topic");
      //ROS_INFO("Current topic is: %s", topic);

      //const planner::currentCVTask& temp = event.getMessage(); //store in global variable
      //ROS_INFO("Current task is %i", currentCVTask_Front.currentCVTask);
    } 
    */


void currentCVTask_Front_callback(planner::currentCVTask msg)
{
  currentCVTask_Front = msg;
}

computer_vision::VisibleObjectData tf2CV(const std::string& targetFrame,const std::string& originalFrame)
{
  /*
  * Looks up the TF and wraps the data in a CV object.
  * @param targetFrame find the pose of the originalFrame in this frame
  * @param originalFrame find the pose of this frame in the targetFrame
  */
  tf::StampedTransform transform;
  tf::TransformListener tf_listener;
  computer_vision::VisibleObjectData msg;
    
  try
  {
    tf_listener.waitForTransform(targetFrame, originalFrame, ros::Time(0), ros::Duration(0.4)); //not sure what an appropriate time to wait is. I wanted to wait less than the target 2 Hz.
    tf_listener.lookupTransform(targetFrame, originalFrame, ros::Time(0), transform);
  }
  catch (tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
  
  msg.x_distance = transform.getOrigin().x();
  msg.y_distance = transform.getOrigin().y();
  msg.z_distance = transform.getOrigin().z();

  tf::Quaternion q = transform.getRotation(); //save the rotation as a quaternion
  tf::Matrix3x3 m(q); //convert quaternion to matrix

  double roll; //unused, but needs to be sent to getRPY method
  
  m.getRPY(roll, msg.pitch_angle, msg.yaw_angle); //get rpy from matrix
  ROS_INFO("RPY: %f %f %f", roll, msg.pitch_angle, msg.yaw_angle);

  return msg;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "distanceCalculator");

  ros::NodeHandle node;
  ros::Subscriber currentCVTask_Front_sub = node.subscribe("currentCVTask_Front", 1000, currentCVTask_Front_callback);

  ros::Publisher cv_mock_pub_Front = node.advertise<computer_vision::VisibleObjectData>("/front_cv_data", 1000);

  computer_vision::VisibleObjectData msgCV;
  
  ros::Rate loop_rate(CV_PUBLISHING_RATE);
  while (node.ok()){
    ros::spinOnce();
    msgCV = tf2CV("camera1_reoriented", "gate_center_sim");
    cv_mock_pub_Front.publish(msgCV);
    loop_rate.sleep();
    ROS_INFO("Current Time: %f", ros::Time::now().toSec());
  }
  return 0;
}