#include "FrontCameraNode.h"


int main(int argc, char **argv) {

	if (argc == 3) {
		// Initialize the node
		ros::init(argc, argv, FRONT_CAMERA_NODE_NAME);
		ros::NodeHandle nodeHandle;

		ROS_INFO("Initializing node %s", ros::this_node::getName().c_str());

		// Create a new CameraNode object
		FrontCameraNode* pFrontCameraNode = new FrontCameraNode(nodeHandle,
															    FRONT_CAMERA_NODE_TOPIC,
															    FRONT_CAMERA_NODE_TRANSMISSION_RATE,
															    FRONT_CAMERA_NODE_BUFFER_SIZE,
															    argv[1],
															    argv[2]);

		// Start sending images to computer vision node (subscriber)
		pFrontCameraNode->sendImages();

		// Destroy the CameraNode object
		delete pFrontCameraNode;

		// Destroy the node
		ros::shutdown();
	}
	else
		ROS_ERROR("Usage: rosrun computer_vision FrontCameraNode [left video source] [right video source]");
}

/**
 * @brief Constructor.
 *
 * Constructs a new DownCameraNode object (calls the CameraNode constructor).
 *
*/
FrontCameraNode::FrontCameraNode(ros::NodeHandle& nodeHandle, const std::string topicName, int transmissionRate, int bufferSize, char* leftCaptureSource, char* rightCaptureSource) : CameraNode(nodeHandle, topicName, transmissionRate, bufferSize) {
	this->pLeftCamera = new Camera(leftCaptureSource);
	this->pRightCamera = new Camera(rightCaptureSource);
}

/**
 * @brief Destructor.
 *
 * Releases the memory used by the DownCameraNode object.
 *
*/
FrontCameraNode::~FrontCameraNode() {
	delete pLeftCamera;
	delete pRightCamera;
}

/**
 * Takes a frame from the down camera and sends it to the down cv node.
 */
void FrontCameraNode::sendImages() {
	ros::Rate loop_rate(transmissionRate);

	while (ros::ok()) {
		// Capture a new frame
		pLeftCamera->captureFrame();
		pRightCamera->captureFrame();

		// TODO: Pack both images into a single ros message before publishing them

		// Publish the new frame
		try {
			// Convert the cv_bridge::CvImage to ROS image messages and publish them
			cvNodePublisher.publish(toCvImage(pLeftCamera->getLastFrame()).toImageMsg());
			//cvNodePublisher.publish(toCvImage(pRightCamera->getLastFrame()).toImageMsg());
		}
		catch (cv_bridge::Exception& e) {
			ROS_ERROR("CvBridge exception in DownCameraNode::sendImages(): %s", e.what());
			return;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}
