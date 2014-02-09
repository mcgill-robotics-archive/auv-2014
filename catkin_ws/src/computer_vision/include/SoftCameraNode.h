#include "CameraNode.h"

const std::string SOFT_CAMERA_NODE_NAME = "soft_camera_node";

const int SOFT_CAMERA_NODE_TRANSMISSION_RATE = 10;
const int SOFT_CAMERA_NODE_BUFFER_SIZE = 1;


class SoftCameraNode : CameraNode {

	private:
	cv::VideoCapture videoSource;

	public:

	SoftCameraNode(ros::NodeHandle& nodeHandle, const std::string topicName, const cv::VideoCapture& video);
	~SoftCameraNode();
	void sendImages();

};

