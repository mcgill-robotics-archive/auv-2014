/**
* Listens to TF between robot and objects and publishes fake CV data
* used in simulation to replace CV for unit tests or to validate CV
*@author Nick Speal
*@modified March 9
*/

#include "distanceCalculator.h"

int CV_PUBLISHING_RATE = 2; //publish at 2 Hz, just like CV
planner::currentCVTask currentCVTask_Front;

void currentCVTask_Front_callback(planner::currentCVTask msg)
{
  currentCVTask_Front = msg;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "distanceCalculator");

  ros::NodeHandle node;
  ros::Subscriber currentCVTask_Front_sub = node.subscribe("currentCVTask_Front", 1000, currentCVTask_Front_callback);

  ros::Publisher cv_mock_pub_Front = node.advertise<computer_vision::VisibleObjectData>("/front_cv_data", 1000);

  tf::TransformListener tf_listener;

  ros::Rate loop_rate(CV_PUBLISHING_RATE);
  while (node.ok()){
    ros::spinOnce();
    tf::StampedTransform transform;
    
    try
    {
        tf_listener.waitForTransform("camera1_reoriented", "gate_center_sim", ros::Time(0), ros::Duration(10.0));
        tf_listener.lookupTransform("camera1_reoriented", "gate_center_sim", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
    
    computer_vision::VisibleObjectData msgCV;
    msgCV.x_distance = transform.getOrigin().x();
    msgCV.y_distance = transform.getOrigin().y();
    msgCV.z_distance = transform.getOrigin().z();

    cv_mock_pub_Front.publish(msgCV);
    loop_rate.sleep();
  }
  return 0;
}