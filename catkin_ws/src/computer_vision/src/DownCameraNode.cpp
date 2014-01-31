#include "DownCameraNode.h"


int main(int argc, char **argv) {

	if (argc == 2) {
		// Initialize the node
		ros::init(argc, argv, DOWN_CAMERA_NODE_NAME);
		ros::NodeHandle nodeHandle;

		ROS_INFO("Initializing node %s", ros::this_node::getName().c_str());

		// Create a new CameraNode object
		DownCameraNode* pDownCameraNode = new DownCameraNode(nodeHandle,
															 DOWN_CAMERA_NODE_TOPIC,
															 DOWN_CAMERA_NODE_TRANSMISSION_RATE,
															 DOWN_CAMERA_NODE_BUFFER_SIZE,
															 argv[1]);

		// Start sending images to computer vision node (subscriber)
		pDownCameraNode->sendImages();

		// Destroy the CameraNode object
		delete pDownCameraNode;

		// Destroy the node
		ros::shutdown();
	}
	else
		ROS_ERROR("Usage: rosrun computer_vision DownCameraNode [video source]");
}

/**
 * @brief Constructor.
 *
 * Constructs a new DownCameraNode object (calls the CameraNode constructor).
 *
*/
DownCameraNode::DownCameraNode(ros::NodeHandle& nodeHandle, const std::string topicName, int transmissionRate, int bufferSize, char* captureSource) : CameraNode(nodeHandle, topicName, transmissionRate, bufferSize) {
	this->pDownCamera = new Camera(captureSource);
}

/**
 * @brief Destructor.
 *
 * Releases the memory used by the DownCameraNode object.
 *
*/
DownCameraNode::~DownCameraNode() {
	delete pDownCamera;
}

/**
 * Takes a frame from the down camera and sends it to the down cv node.
 */
void DownCameraNode::sendImages() {
	ros::Rate loop_rate(transmissionRate);

	while (ros::ok()) {
		// Capture a new frame
		pDownCamera->captureFrame();

		// Publish the new frame
		try {
			// Convert the cv_bridge::CvImage to ROS image messages and publish them
			cvNodePublisher.publish(toCvImage(pDownCamera->getLastFrame()).toImageMsg());
		}
		catch (cv_bridge::Exception& e) {
			ROS_ERROR("CvBridge exception in DownCameraNode::sendImages(): %s", e.what());
			return;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}
