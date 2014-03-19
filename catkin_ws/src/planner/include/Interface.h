#ifndef Interface_h
#define Interface_h

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
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
#include <tf/transform_listener.h>
#include "planner/CurrentCVTask.h"
#include <vector>
#include <cmath>
#include <boost/thread.hpp>

void spinThread();

void estimatedState_callback(const computer_vision::VisibleObjectData data);

void estimatedDepth_callback(const std_msgs::Float64 data);

void setTransform (std::string referenceFrame);

void setPose();

bool areWeThereYet(std::vector<double> desired);

bool areWeThereYet_tf(std::string referenceFrame);

void setVisionObj (int objIndex);

void weAreHere (std::string task);

void setPoints (double pointControl[]);

void setVelocity (double x_speed, double y_speed, double yaw_speed, double depth);

//void setPosition (double x_pos, double y_pos, double pitch_angle, double yaw_angle, double depth);
void setPosition (std::vector<double> desired);

void rosSleep(int length);

int main (int argc, char **argv);

void ps3Control (); //LEGACY -- NOT FOR TOUCHING

#endif // Interface_h
