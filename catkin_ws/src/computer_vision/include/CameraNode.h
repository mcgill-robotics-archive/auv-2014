#ifndef CV_CAMERA_NODE_H
#define CV_CAMERA_NODE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>

#include "Camera.h"

class CameraNode {

	private:

	image_transport::ImageTransport* pImageTransport;

	protected:

	int transmissionRate;
	image_transport::Publisher cvNodePublisher;
	cv_bridge::CvImage toCvImage(cv::Mat* pFrame);

	public:
	
	CameraNode(ros::NodeHandle& nodeHandle, const std::string topicName, int transmissionRate, int bufferSize);
	virtual ~CameraNode();
	virtual void sendImages() = 0;
};

#endif
