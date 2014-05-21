#ifndef THRUST_MAPPER_H
#define THRUST_MAPPER_H
#include "ros/ros.h"
#include <ros/console.h> //to change verbosity of ROSINFO ROS_DEBUG etc

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Wrench.h"
#include "std_msgs/Float64.h"
#include "robosub_msg/setPoints.h"	
#include "computer_vision/VisibleObjectData.h"
#include "controls/DebugControls.h"
#include "gazebo_msgs/ModelStates.h"
#include "controls/motorCommands.h"
#include <string>

float limit_check(float value, float min, float max, char* value_type, char* value_id );
void thrust_callback(geometry_msgs::Wrench wrenchMsg);
float thrust_voltage(float thrust);
#endif