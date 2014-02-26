#ifndef Interface_h
#define Interface_h

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "simulator/ThrusterForces.h"
#include "planner/setPoints.h"
#include "Loader.h"
#include "Invoker.h"
#include "computer_vision/VisibleObjectData.h"
#include "gazebo_msgs/ModelStates.h"

void setVisibleObjectOrientation (computer_vision::VisibleObjectData msg);

//BEWARE: RETURNS ADDRESS OF ARRAY
double* getVisibleObjectOrientation ();

void setOurOrientation (gazebo_msgs::ModelStates msg);

//BEWARE: RETURNS ADDRESS OF ARRAY
double* getOurOrientation ();

void setDepth(std_msgs::Float32 msg);

bool isCorrectDepth(double desiredDepth);

void setVisionObj (int objIndex);

void weAreHere (std::string task);

void setPoints (double pointControl[]);

void setVelocity (double x_speed, double y_speed, double yaw_speed, double depth);

void setPosition (double x_pos, double y_pos, double pitch_angle, double yaw_angle, double depth);

void rosSleep(int length);

int main (int argc, char **argv);

void ps3Control (); //LEGACY -- NOT FOR TOUCHING

#endif // Interface_h
