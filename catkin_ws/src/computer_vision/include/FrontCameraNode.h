#include "CameraNode.h"

const std::string FRONT_CAMERA_NODE_NAME = "front_camera_node";
const std::string FRONT_CAMERA_NODE_TOPIC = "/simulator/camera1/image_raw";

const int FRONT_CAMERA_NODE_TRANSMISSION_RATE = 10;
const int FRONT_CAMERA_NODE_BUFFER_SIZE = 1;


class FrontCameraNode : CameraNode {

	private:

	Camera* pLeftCamera;
	Camera* pRightCamera;

	public:

	FrontCameraNode(ros::NodeHandle& nodeHandle, const std::string topicName, int transmissionRate, int bufferSize, char* leftCaptureSource, char* rightCaptureSource);
	~FrontCameraNode();
	void sendImages();

};

