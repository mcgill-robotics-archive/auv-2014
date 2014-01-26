#include "ros/ros.h"
#include "gazebo_msgs/ModelStates.h"
#include "std_msgs/Float64.h"



// global vars

double z_position;


void estimatedDepth_callback(gazebo_msgs::ModelStates data) // subscribe to model states
  {
    z_position = data.pose[0].position.z;
}

int main(int argc, char **argv)
{


// create node called "depthPublisher"
  ros::init(argc, argv, "depthPublisher");
  ros::NodeHandle n;

  // parameters

  std_msgs::Float64 depthVal;
  double surfaceHeight;
  n.param<double>("surfaceHeight", surfaceHeight, 10);

  ros::Publisher depthPub = n.advertise<std_msgs::Float64>("depthCalculated", 1000); // publish to topic called "depthCalculated"

  ros::Rate loop_rate(10);

  while (ros::ok())
  {

    depthVal.data = surfaceHeight - z_position;

    depthPub.publish(depthVal);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}