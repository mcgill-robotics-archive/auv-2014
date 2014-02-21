#ifndef FRONT_CV_NODE_H
#define FRONT_CV_NODE_H

#include "CVNode.h"
#include "Buoy.h"
#include "Gate.h"


//#include "computer_vision/StereoCameraImages.h"

const std::string FRONT_CV_NODE_NAME = "front_cv_node";

const std::string CAMERA1_CV_TOPIC_NAME = "front_cv_camera1";
const std::string DATA_TOPIC_NAME = "front_cv_data";

const int FRONT_CV_NODE_RECEPTION_RATE = 10;
const int FRONT_CV_NODE_BUFFER_SIZE = 1;

class FrontCVNode : public CVNode {

	public:

	FrontCVNode(ros::NodeHandle& nodeHandle, std::string topicName, int receptionRate, int bufferSize);
	~FrontCVNode();

	private:

	cv::Mat* pLastImageLeftCamera;
	cv::Mat* pLastImageRightCamera;

	void receiveImage(const sensor_msgs::ImageConstPtr& message);
};

#endif
