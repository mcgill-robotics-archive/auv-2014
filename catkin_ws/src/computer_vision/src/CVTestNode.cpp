/**
 * @file CVNode.cpp
 * @author Renaud Dagenais
 * @author Jean-Sebastien Dery
 * @author Haris Haidary
 * @version 1.0.0
 * @date December 17th 2013
 * @brief TODO since it will be renamed and its purpose will change a bit lulzy lulzo.
*/
#include "CVTestNode.h"

/**
 * Defines the number of seconds between every delays when the node is waiting for someone to publish the topic.
 */
#define DELAY_BETWEEN_INFOS 5

/**
 * @brief Main method used by ROS when the node is launched.
 *
 * @param argc The number of arguments passed when the process is stared.
 * @param argv The arguments passed when the process is started.
 * @return The termination status of the processe's execution.
 */
int main(int argc, char **argv) {
	ros::init(argc, argv, "cv_node");
	ros::NodeHandle nodeHandle;

	ROS_INFO("%s", "Starting CVTestNode.");

	// Creates a new CVNode object.
	CVTestNode* pCVTestNode = new CVTestNode(nodeHandle);

	// Start receiving images from the camera node (publisher)
	while (ros::ok()) {
		;
	}

	// Destroy the CVNode object
	delete pCVTestNode;

	// Destroy the node
	ros::shutdown();
}

/**
 * @brief Constructor.
 *
 * Constructs a new CVNode object. The computer vision node receives
 * images from the camera node and transfers them to each VisibleObject.
 *
 * @param nodeHandle The ROS node handle
 * @param topicName The name of the topic on which the images are published
 */
CVTestNode::CVTestNode(ros::NodeHandle& nodeHandle) {
	ROS_INFO("%s", "The subscriber has been configured, now about to wait for a node to publishing the camera feed.");

	// Wait for publisher(s) to be ready.
	while (true) {
		ROS_INFO_THROTTLE(DELAY_BETWEEN_INFOS, "Now waiting for a node to publish a video feed...");
	}
}

/**
 * @brief Destructor.
 *
 * Releases the memory used by the CVNode object.
 */
CVTestNode::~CVTestNode() {

}
