#ifndef CV_CAMERA_NODE_H
#define CV_CAMERA_NODE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc/imgproc.hpp>

#include "Camera.h"

class CameraNode {

	private:

	int transmissionRate;
	Camera* pCamera;
	image_transport::ImageTransport* pImageTransport;
	image_transport::Publisher publisher;
	cv_bridge::CvImage* pLastImage;
	
	public:
	
	CameraNode(ros::NodeHandle& nodeHandle, const char* topicName, const char* captureSource, int transmissionRate);
	~CameraNode();
	void sendImage();

	private:

	void encodeFrame(cv::Mat* pFrame);
};

#endif
