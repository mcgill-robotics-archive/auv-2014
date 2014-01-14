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
 * Defines the number of seconds between every delays when the node is waiting for someone to publish the topic.
 */
#define DELAY_BETWEEN_INFOS 5

/**
 * Defines what is the name of the topic on which the node will be listening to for the image feeds.
 */
const std::string VIDEO_FEED_TOPIC_NAME = "camera_feed";

/**
 * Defines the topic name used by the planner to control what object the forward cameras will be searching for.
 */
const std::string FORWARD_CAMERAS_TOPIC_NAME = "forward_cameras_object";

/**
 * @brief Constructor.
 *
 * Constructs a new CVNode object. The computer vision node receives
 * images from the camera node and transfers them to each VisibleObject.
 *
 * @param nodeHandle The ROS node handle
 * @param topicName The name of the topic on which the images are published
 */
CVNode::CVNode(ros::NodeHandle& nodeHandle, std::list<std::string> topicList, int receptionRate) {
	this->receptionRate = receptionRate;
	this->pImageTransport = new image_transport::ImageTransport(nodeHandle);

	// Instanciate the subscribers
	for (std::list<std::string>::iterator it = topicList.begin(); it != topicList.end(); it++) {
		// boost::bind() is used to pass an argument (the topic name) to the callback function (CVNode::receiveImage)
		image_transport::Subscriber subscriber = pImageTransport->subscribe(*it, 1, boost::bind(&CVNode::receiveImage, this, _1, *it), ros::VoidPtr(), image_transport::TransportHints());
		this->subscribers.push_back(subscriber);

		// Create a window for that topic
		cv::namedWindow(*it, CV_WINDOW_KEEPRATIO);
	}

	ROS_INFO("%s", "Subscriber(s) configured.");

	// Wait for publisher(s) to be ready
	for (std::list<image_transport::Subscriber>::iterator it = subscribers.begin(); it != subscribers.end(); it++) {
		while ((*it).getNumPublishers() == 0) {
			std::string message = "Waiting for a publisher to start publishing on the topic ";
			message += (*it).getTopic();
			ROS_INFO_THROTTLE(DELAY_BETWEEN_INFOS, "%s", message.c_str());
		}
	}

	ROS_INFO("%s", "Publisher(s) ready.");
}

/**
 * @brief Destructor.
 *
 * Releases the memory used by the CVNode object.
 */
CVNode::~CVNode() {
	// Unsubscribe from all topics and destroy windows
	for (std::list<image_transport::Subscriber>::iterator it = subscribers.begin(); it != subscribers.end(); it++) {
		(*it).shutdown();
		cv::destroyWindow(getTopicName(*it));
	}

	// Delete all VisibleObjects
	while (!visibleObjects.empty()) {
		delete visibleObjects.front();
		visibleObjects.pop_front();
	}

	// Delete ImageTransport object
	delete pImageTransport;
}

/**
 * @brief Function that dictates the rate at which the node checks if an image is received.
 */
void CVNode::receiveImages() {

	ROS_INFO("%s", (ros::this_node::getName() + " will start to receive images from publisher.").c_str());

	ros::Rate loop_rate(receptionRate);

	while (ros::ok()) {
		if (subscribers.empty()) {
			// If the node is not subscribed to any topic then stop receiving images
			break;
		}
		else {
			// Check if there are still publishers on each topics
			for (std::list<image_transport::Subscriber>::iterator it = subscribers.begin(); it != subscribers.end(); it++) {
				if ((*it).getNumPublishers() == 0) {
					// If there is no publisher on a topic, stop listening to it and destroy the window that displays the images
					(*it).shutdown();
					cv::destroyWindow(getTopicName(*it));
					subscribers.remove(*it);
					break;
				}
			}

			ros::spinOnce();
			loop_rate.sleep();
		}
	}
}

/**
 * Returns the name of the topic a subscriber is subscribed to.
 */
std::string getTopicName(image_transport::Subscriber subscriber) {
	return subscriber.getTopic().erase(0, 1);
}

