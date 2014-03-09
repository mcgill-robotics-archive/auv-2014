/**
* Listens to TF between robot and objects and publishes fake CV data
* used in simulation to replace CV for unit tests or to validate CV
*@author Nick Speal
*@modified March 7
*/


#include "distanceCalculator.h"
#include <tf/transform_listener.h>


int CV_PUBLISHING_RATE = 2; //publish at 2 Hz, just like CV


double X; double Y; double Z;
double Yaw; double Pitch;
double x_dist; double y_dist; double z_dist;
double x_dist_global; double y_dist_global;
double robot_x; double robot_y; double robot_z;
double gate_x; double gate_y; double gate_z;

planner::currentCVTask currentCVTask_Front;
planner::currentCVTask currentCVTask_Down;


/*

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
void currentCVTask_Down_callback(planner::currentCVTask msg)
{
  currentCVTask_Down = msg;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "distanceCalculator");
  ros::NodeHandle n;

  //ros::Subscriber currentCVTask_Front_sub = n.subscribe("currentCVTask_Front", 1000, currentCVTask_callback);
  ros::Subscriber currentCVTask_Front_sub = n.subscribe("currentCVTask_Front", 1000, currentCVTask_Front_callback);
  ros::Subscriber currentCVTask_Down_sub = n.subscribe("currentCVTask_Down", 1000, currentCVTask_Down_callback);

  ros::Publisher cv_mock_pub_Front = n.advertise<computer_vision::VisibleObjectData>("/front_cv_data", 1000);
  ros::Publisher cv_mock_pub_Down = n.advertise<computer_vision::VisibleObjectData>("/Down_cv_data", 1000);

  ros::Rate loop_rate(CV_PUBLISHING_RATE); 

  //tf::TransformListener tf_listener;

  while (ros::ok())
  {
    ros::spinOnce();
    
    //tf::StampedTransform transform;

  	if (currentCVTask_Front.currentCVTask == currentCVTask_Front.GATE)
  	{
      try
      {
        //tf_listener.lookupTransform("camera1_reoriented", "gate_center_sim", ros::Time(0), transform);
        int dummy;
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
      }
  	}
  	else if (currentCVTask_Front.currentCVTask == currentCVTask_Front.NOTHING)
  	{
  		ROS_INFO("Not looking for anything in front");
  		loop_rate.sleep();
      continue;
  	}
  	else
  	{
  		ROS_WARN("unrecognized Current CV Task _ Front: %i", currentCVTask_Front.currentCVTask);
      loop_rate.sleep();
      continue;
  	}
    //ROS_INFO("Gate y: %f | robot y: %f", gate_y, robot_y);
  	//wrap distances into CV message
    computer_vision::VisibleObjectData msgCV;
    //msgCV.x_distance = transform.getOrigin().x();
    //msgCV.y_distance = transform.getOrigin().y();
    //msgCV.z_distance = transform.getOrigin().z();

    double roll11;
    double pitch11;
    double yaw11;
    //tf::Quaternion q = transform.getRotation(); //save the rotation as a quaternion
    //tf::Matrix3x3 m(q); //transform quat to matrix


    //m.getRPY(roll11, pitch11, roll11); //get rpy from matrix
    //ROS_INFO("RPY: %f %f %f", roll11, pitch11, yaw11);
    //msgCV.yaw_angle = Yaw;
    //msgCV.pitch_angle = Pitch;

    //cv_mock_pub_Front.publish(msgCV);
    loop_rate.sleep(); //sleep at the end to make sure we loop at the desired rate. Sleep at end not beginning because it will automatically account for fluctiations in execution time for each loop
  }

  return 0;
}