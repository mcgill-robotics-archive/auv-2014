#include "FrontCVNode.h"

/**
 * @brief Main method used by ROS when the node is launched.
 *
 * @param argc The number of arguments passed when the process is stared.
 * @param argv The arguments passed when the process is started.
 * @return The termination status of the processe's execution.
 */
int main(int argc, char **argv) {

	if (argc > 1) {
		ros::init(argc, argv, "front_cv_node");
		ros::NodeHandle nodeHandle;

		// Create the list of topics to listen to
		std::list<std::string> topicList;
		for (int i = 1; i < argc; i++) {
			std::string topic = std::string(argv[i]);
			topicList.push_back(topic);
		}

		/*
		ROS_INFO("%s", ("Initializing the CVNode. It will be listening to the topic named \"" + VIDEO_FEED_TOPIC_NAME + "\"").c_str());
		ROS_INFO("%s", "If you want to run this node in Gazebo, the topic needs to be renamed to \"/my_robot/camera1/image_raw\"");
		 */

		// Creates a new CVNode object.
		FrontCVNode* pFrontCVNode = new FrontCVNode(nodeHandle, topicList, 1);

		// Start receiving images from the camera node (publisher)
		pFrontCVNode->receiveImages();

		// Destroy the CVNode object
		delete pFrontCVNode;

		// Destroy the node
		ros::shutdown();
	}
	else
		std::cout << "Usage: rosrun computer_vision FrontCVNode [topic 1] [topic 2] ... [topic n]" << std::endl;
}

/**
 * @brief Constructor.
 *
 * Constructs a new FrontCVNode object (calls the CVNode constructor).
 *
 */
FrontCVNode::FrontCVNode(ros::NodeHandle& nodeHandle, std::list<std::string> topicList, int receptionRate) : CVNode(nodeHandle, topicList, receptionRate) {
	// Construct the list of VisibleObjects
	visibleObjects.push_back(new Door());
}


/**
 * @brief Function that is called when an image is received.
 *
 * @param rosMessage The ROS message that contains the image
 */
void FrontCVNode::receiveImage(const sensor_msgs::ImageConstPtr& message, const std::string &topicName) {
	cv_bridge::CvImagePtr pCurrentFrame;
	cv::Mat currentFrame;
	std::list<VisibleObject*>::iterator it;

	try {
		// Convert sensor_msgs to an opencv image
		pCurrentFrame = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
		currentFrame = pCurrentFrame->image;
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	try {
		// Loop through the list of visible objects and transmit
		// the received image to each visible object
		for (it = visibleObjects.begin(); it != visibleObjects.end(); it++) {
			(*it)->retrieveObjectData(currentFrame);
		}

		// Display the filtered image
		cv::imshow(topicName, currentFrame);
		cv::waitKey(5);
	} catch (cv::Exception& e) {
		ROS_ERROR("cv::imgshow exception: %s", e.what());
		return;
	}
}