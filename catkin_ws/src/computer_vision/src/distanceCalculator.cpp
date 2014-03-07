/**
* Listens to TF between robot and objects and publishes fake CV data
*@author Nick Speal
*@modified March 7
*/


#include "distanceCalculator.h"
double X; double Y; double Z;
double Yaw; double Pitch;
double x_dist; double y_dist; double z_dist;
double x_dist_global; double y_dist_global;
double robot_x; double robot_y; double robot_z;
double gate_x; double gate_y; double gate_z;

planner::currentCVTask currentCVTask_Front;
//planner::currentCVTask currentCVTask_Down;



void currentCVTask_callback(const ros::MessageEvent<planner::currentCVTask const>& event)
{
  const ros::M_string& header = event.getConnectionHeader();
  std::string topic = header.at("topic");
  //ROS_INFO("Current topic is: %s", topic);

  //const planner::currentCVTask& temp = event.getMessage(); //store in global variable
  //ROS_INFO("Current task is %i", currentCVTask_Front.currentCVTask);
} 
/*
void currentCVTask_Front_callback(planner::currentCVTask msg)
{
  currentCVTask_Front = msg;
}
*/
/*
Uncomment once the Front version works
void currentCVTask_Down_callback(std_msgs::String data)
{
	currentCVTask_Down = data.data;
}
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "distanceCalculator");


  ros::NodeHandle n;

  //ros::Subscriber CV_sub = n.subscribe("gazebo/model_states", 1000, setGazeboPretendThings);
  ros::Subscriber currentCVTask_Front_sub = n.subscribe("currentCVTask_Front", 1000, currentCVTask_callback);
  //ros::Subscriber currentCVTask_Front_sub = n.subscribe("currentCVTask_Front", 1000, currentCVTask_Front_callback);
  //ros::Subscriber currentCVTask_Down_sub = n.subscribe("currentCVTask_Down", 1000, currentCVTask_Down_callback); //uncomment once the front version works

  ros::Publisher pose_pub = n.advertise<computer_vision::VisibleObjectData>("/front_cv_data", 1000);

  ros::Rate loop_rate(10);
  int count = 0;

  //Parameters

  double surfaceHeight;
  n.param<double>("surfaceHeight", surfaceHeight, 10.0);




  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();

  	if (currentCVTask_Front.currentCVTask == currentCVTask_Front.GATE)
  	{
      //ROS_INFO("Current CV Task Front = GATE");
      //compute distances here
  		x_dist_global = gate_x - robot_x;
  		y_dist_global = gate_y - robot_y;
  		z_dist = gate_z - robot_z;


  		//TODO Add Rotation of Coordinates here
  		x_dist = x_dist_global;
  		y_dist = y_dist_global;
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
    	//wrap distanecs into CV message
      computer_vision::VisibleObjectData msgCV;
      msgCV.x_distance = x_dist;
      msgCV.y_distance = y_dist;
      msgCV.z_distance = z_dist;
      msgCV.yaw_angle = Yaw;
      msgCV.pitch_angle = Pitch;

      pose_pub.publish(msgCV);
    
  }

  return 0;
}
