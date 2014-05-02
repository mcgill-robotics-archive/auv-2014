#ifndef FRONT_CV_NODE_H
#define FRONT_CV_NODE_H

#include "CVNode.h"
#include "Buoy.h"
#include "Gate.h"
#include "LineTarget.h"

// These variables need to be accessible from the main method in FrontCVNode.cpp.
const std::string FRONT_CV_NODE_NAME = "front_cv";
const std::string CAMERA1_CV_TOPIC_NAME = "front_cv/camera1";
const std::string CAMERA2_CV_TOPIC_NAME = "front_cv/camera2";
const std::string OUTPUT_DATA_TOPIC_NAME = "front_cv/data";
const std::string PLANNER_DATA_FRONT_TOPIC_NAME = "currentCVTask_Front";
const int FRONT_CV_NODE_RECEPTION_RATE = 10;
const int FRONT_CV_NODE_BUFFER_SIZE = 1;
bool isUsingHelperWindows;

class FrontCVNode : public CVNode {

public:

	FrontCVNode(ros::NodeHandle& nodeHandle, std::string topicName, int receptionRate, int bufferSize);
	~FrontCVNode();

private:
	int numFramesWithoutObject;

	void instanciateAllVisibleObjects();
	void receiveImage(const sensor_msgs::ImageConstPtr& message);
	void listenToPlanner(planner::CurrentCVTask msg);
};

#endif
