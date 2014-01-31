/**
 * @file CVNode.cpp
 * @author Renaud Dagenais
 * @author Jean-Sebastien Dery
 * @author Haris Haidary
 * @version 1.0.0
 * @date December 17th 2013
 * @brief TODO since it will be renamed and its purpose will change a bit lulzy lulzo.
*/
#include "CVNode.h"

/**
 * @brief Constructor.
 *
 * Constructs a new CVNode object. The computer vision node receives
 * images from the camera node and transfers them to each VisibleObject.
 *
 * @param nodeHandle The ROS node handle
 * @param topicName The name of the topic on which the images are published
 */
CVNode::CVNode(ros::NodeHandle& nodeHandle, std::string topicName, int receptionRate, int bufferSize) {
	this->receptionRate = receptionRate;
	this->pImageTransport = new image_transport::ImageTransport(nodeHandle);

	// Instanciate the subscribers
	cameraNodeSubscriber = pImageTransport->subscribe(topicName, bufferSize, &CVNode::receiveImage, this);

	// Wait for publisher(s) to be ready
	while (cameraNodeSubscriber.getNumPublishers() == 0) {
		ROS_INFO_THROTTLE(DELAY_BETWEEN_INFOS, "Waiting for a publisher to start publishing on the topic /%s", topicName.c_str());
	}
}

/**
 * @brief Destructor.
 *
 * Releases the memory used by the CVNode object.
 */
CVNode::~CVNode() {
	// Delete all VisibleObjects
	while (!visibleObjectList.empty()) {
		delete visibleObjectList.front();
		visibleObjectList.pop_front();
	}

	frontEndPublisher.shutdown();

	delete pImageTransport;
}

/**
 * @brief Function that dictates the rate at which the node checks if an image is received.
 */
void CVNode::receiveImages() {
	ROS_INFO("%s", (ros::this_node::getName() + " will start to receive images from publisher.").c_str());

	ros::Rate loop_rate(receptionRate);

	while (ros::ok()) {
		// Check if the camera is still publishing
		if (cameraNodeSubscriber.getNumPublishers() == 0) {
			break;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}

