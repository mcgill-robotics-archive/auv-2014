#ifndef FRONT_CV_NODE_H
#define FRONT_CV_NODE_H

#include "CVNode.h"
#include "Door.h"

class FrontCVNode : public CVNode {

	public:

	FrontCVNode(ros::NodeHandle& nodeHandle, std::list<std::string> topicList, int receptionRate);

	private:

	void receiveImage(const sensor_msgs::ImageConstPtr& message, const std::string &topicName);
};

#endif
