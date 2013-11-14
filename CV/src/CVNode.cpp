#include "CVNode.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "cv_node");
    ros::NodeHandle nodeHandle;
    
    // Creates a new CVNode object
    CVNode* pCVNode = new CVNode(nodeHandle, "camera_feed");

    // Start receiving images from the camera node (publisher)
    while (ros::ok()) {

    	// Check if the camera node is still sending images
    	if (pCVNode->getNumPublisher() > 0)
    		ros::getGlobalCallbackQueue()->callAvailable();
    	else
    		break;
    }

    // Destroy the CVNode object
    delete pCVNode;

    // Destroy the node
	ros::shutdown();
}

/**
 * Constructs a new CVNode object. The computer vision node receives
 * images from the camera node and transfers them to each VisibleObject.
 * @param nodeHandle The ROS node handle
 * @param topicName The name of the topic on which the images are published
 */
CVNode::CVNode(ros::NodeHandle& nodeHandle, const char* topicName) {
	// Subscribe to the "camera_feed" topic
	pImageTransport = new image_transport::ImageTransport(nodeHandle);
	subscriber = pImageTransport->subscribe(topicName, 1, &CVNode::receiveImage, this);
	
	// Construct the list of VisibleObjects
	visibleObjects.push_back(new Door());

	// Setup a window to display the camera feed
	cv::namedWindow(MAIN_WINDOW, CV_WINDOW_KEEPRATIO);
	cv::namedWindow(FILTERED_WINDOW, CV_WINDOW_KEEPRATIO);
	cv::moveWindow(MAIN_WINDOW, 100, 30);
	cv::moveWindow(FILTERED_WINDOW, 500, 30);

	// Wait for publisher(s) to be ready
	while (subscriber.getNumPublishers() == 0);
}

/**
 * Releases the memory used by the CVNode object.
 */
CVNode::~CVNode() {
	// Destroy window
	cv::destroyWindow(MAIN_WINDOW);

	// Unsubscribe from topic
	subscriber.shutdown();
	delete pImageTransport;

	// Empty the list of VisibleObjects
	while(!visibleObjects.empty()) {
		delete visibleObjects.front();
		visibleObjects.pop_front();
	}
}

/**
 * Function that is called when an image is received.
 * @param rosMessage The ROS message that contains the image
 */
void CVNode::receiveImage(const sensor_msgs::ImageConstPtr& message) {
	cv_bridge::CvImagePtr pCurrentFrame;
	cv::Mat currentFrame;
	std::list<VisibleObject*>::iterator it;

	try {
		// Convert sensor_msgs to an opencv image
	    pCurrentFrame = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
	    currentFrame = pCurrentFrame->image;
	}
	catch (cv_bridge::Exception& e) {
	    ROS_ERROR("cv_bridge exception: %s", e.what());
	    return;
	}
	
	try {
		// Loop through the list of visible objects and transmit
		// the received image to each visible object
	    for (it = visibleObjects.begin(); it != visibleObjects.end(); it++)
    		(*it)->retrieveObjectData(currentFrame);

		// Display the filtered image
	    cv::imshow(MAIN_WINDOW, currentFrame);
	    cv::waitKey(5);
	}
	catch (cv::Exception& e) {
	    ROS_ERROR("cv::imgshow exception: %s", e.what());
	    return;
	}
}

/**
 * Helper function that is used to tell whether or not the node is
 * still receiving images from the publisher(s).
 * @return The number of publishers on the topic
 */
int CVNode::getNumPublisher() {
	return subscriber.getNumPublishers();
}
