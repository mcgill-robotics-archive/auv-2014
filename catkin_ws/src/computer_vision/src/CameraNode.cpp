#include "CameraNode.h"

/**
 * Constructs a CameraNode object. The camera node is used to send images
 * to the computer vision node.
 * @param nodeHandle The node handle
 * @param topicName The name of the topic on which the images will published
 * @param captureSource The path to the video file to be transmitted (if empty, the default camera is used)
 * @param transmissionRate The rate at which the images are published on the topic
 */
CameraNode::CameraNode(ros::NodeHandle& nodeHandle, const std::string topicName, int transmissionRate, int bufferSize) {
	this->pImageTransport = new image_transport::ImageTransport(nodeHandle);
	this->transmissionRate = transmissionRate;
	this->cvNodePublisher = pImageTransport->advertise(topicName, bufferSize);
}

/**
 * Releases the memory used by the CameraNode object.
 */
CameraNode::~CameraNode() {
	this->cvNodePublisher.shutdown();
	delete this->pImageTransport;
}

/**
 * Helper function used to convert an opencv image to cv_bridge image.
 * @param pFrame Pointer to the opencv image to convert
 */
cv_bridge::CvImage CameraNode::toCvImage(cv::Mat* pFrame) {
	cv_bridge::CvImage cvImage;

	// Set image properties
	cvImage.header.stamp    = ros::Time::now();
	cvImage.encoding        = sensor_msgs::image_encodings::BGR8;
	cvImage.image           = *pFrame;

	return cvImage;
}

