#ifndef PLANNER_SIMULATOR_H
#define PLANNER_SIMULATOR_H

#include <ros/ros.h>
#include "robosub_msg/CurrentCVTask.h"

// These variables need to be accessible from the main method in FrontCVNode.cpp.
const std::string PLANNER_DATA_FRONT_TOPIC_NAME = "currentCVTask_Front";
const std::string PLANNER_DATA_DOWN_TOPIC_NAME = "currentCVTask_Down";

class PlannerSimulator {

public:
	PlannerSimulator(ros::NodeHandle& nodeHandle, std::string topicName, int receptionRate, int bufferSize);
private:
	ros::Publisher plannerPublisherFront;
	ros::Publisher plannerPublisherDown;
};

#endif
