#ifndef Interface_h
#define Interface_h

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "simulator/ThrusterForces.h"
#include "planner/setPoints.h"
#include "Loader.h"
#include "Invoker.h"
#include "computer_vision/VisibleObjectData.h"

void setOrientation(computer_vision::VisibleObjectData msg);

void setPoints(double pointControl[]);

void setPosition(double x_pos, double y_pos, double z_pos, double pitch_angle, double yaw_angle);

void setVelocity(double x_speed, double y_speed, double yaw_speed, double depth);

void ps3Control();

#endif // Interface_h
