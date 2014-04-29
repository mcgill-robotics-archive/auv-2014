#ifndef Interface_h
#define Interface_h

#include "ros/ros.h"
#include "Task_Handler.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/SetModelState.h"
#include "tf/transform_listener.h"
#include "planner/setPoints.h"
#include "planner/CurrentCVTask.h"
#include "blinky/RGB.h"
#include "blinky/UpdatePlannerLights.h"
#include <vector>
#include <cmath>
#include <boost/thread.hpp>

enum BlinkyColors {RED, GREEN, BLUE, WHITE, BLACK, PURPS};

void spinThread();


void estimatedDepth_callback(const std_msgs::Float64 data);

void setTransform (std::string referenceFrame);

std::vector<double> getTransform();

bool areWeThereYet(std::vector<double> desired);

bool areWeThereYet_tf(std::string referenceFrame, std::vector<double> desired);

void setVisionObj (int objIndex);

void weAreHere (std::string task);

void setPoints (double pointControl[]);

void setVelocity (double x_speed, double y_speed, double yaw_speed, double depth);

void setPosition (std::vector<double> desired);

blinky::RGB getColorValues(int myColor);

void updateBlinkyTape(int myColor);

int get_task_id(std::string name);

int main (int argc, char **argv);

#endif // Interface_h
