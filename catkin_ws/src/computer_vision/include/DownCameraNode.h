#include "CameraNode.h"

const std::string DOWN_CAMERA_NODE_NAME = "down_camera_feed";
const std::string DOWN_CAMERA_NODE_TOPIC = "/simulator/camera3/image_raw";

const int DOWN_CAMERA_NODE_TRANSMISSION_RATE = 10;
const int DOWN_CAMERA_NODE_BUFFER_SIZE = 1;


class DownCameraNode : CameraNode {

	private:

	Camera* pDownCamera;

	public:

	DownCameraNode(ros::NodeHandle& nodeHandle, const std::string topicName, int transmissionRate, int bufferSize, char* captureSource);
	~DownCameraNode();
	void sendImages();

};

