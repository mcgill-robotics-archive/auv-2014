#include "FrontCVNode.h"

ros::Publisher visibleObjectDataPublisher;
ros::Publisher frontCVCamera1Publisher;

/**
 * @brief Main method used by ROS when the node is launched.
 *
 * @param argc The number of arguments passed when the process is stared.
 * @param argv The arguments passed when the process is started.
 * @return The termination status of the processe's execution.
 */
int main(int argc, char **argv) {

	ros::init(argc, argv, FRONT_CV_NODE_NAME);
	ros::NodeHandle nodeHandle;

	ROS_INFO("%s", (std::string(__PRETTY_FUNCTION__) + ":: Initializing node '" + ros::this_node::getName() + "'.").c_str());

	std::string imageFeedLeft, imageFeedRight;

	nodeHandle.param<std::string>("image_feed/left", imageFeedLeft, "/simulator/camera1/image_raw");
	nodeHandle.param<std::string>("image_feed/right", imageFeedRight, "/simulator/camera2/image_raw");
	nodeHandle.param<int>("reception_rate", receptionRate, 10);
	nodeHandle.param<int>("image_subscriber_buffer_size", imageSubscriberBufferSize, 0);

	// Creates a new CVNode object.
	FrontCVNode* pFrontCVNode = new FrontCVNode(nodeHandle, imageFeedLeft, receptionRate, imageSubscriberBufferSize);

	// Start receiving images from the camera node (publisher)
	pFrontCVNode->receiveImages();

	// Destroy the CVNode object
	delete pFrontCVNode;

	// Destroy the node
	ros::shutdown();
}

void FrontCVNode::listenToPlanner(planner::CurrentCVTask msg) {
	this->objectsToSearchFor.clear();
	if (msg.currentCVTask == 1) {
		this->objectsToSearchFor.push_back(new Gate(*(this), gp));
	} else if (msg.currentCVTask == 2) {
		objectsToSearchFor.push_back(new Buoy(*(this)));
	}
}

/**
 * @brief Constructor.
 *
 * Constructs a new FrontCVNode object (calls the CVNode constructor).
 *
 */
FrontCVNode::FrontCVNode(ros::NodeHandle& nodeHandle, std::string topicName, int receptionRate, int bufferSize) : CVNode(nodeHandle, topicName, receptionRate, bufferSize) {
	ROS_INFO("%s", (std::string(__PRETTY_FUNCTION__) + ":: initializing FrontCVNode.").c_str());

	// Topics on which the node will be publishing.
	frontEndPublisher = pImageTransport->advertise(CAMERA1_CV_TOPIC_NAME, bufferSize);
	frontEndVisibleObjectDataPublisher = nodeHandle.advertise<computer_vision::VisibleObjectData>(OUTPUT_DATA_TOPIC_NAME, 10);

	// Topics on which the node will be subscribing.
	plannerSubscriber = nodeHandle.subscribe(PLANNER_DATA_FRONT_TOPIC_NAME, 1000, &FrontCVNode::listenToPlanner, this);

	std::string currentObject;
	nodeHandle.param<std::string>("cv_front_detect_object", currentObject, "");
	if (!currentObject.empty()) {
		if (currentObject.compare("gate") == 0) {
			this->objectsToSearchFor.push_back(new Gate(*(this), gp));
		} else if (currentObject.compare("buoy") == 0) {
			// TODO: add the Buoy object here when it is completed
		}
	} else {
		ROS_INFO("%s", "The 'cv_front_detect_object' parameter is empty, the CV will be looking for no object.");
	}

	// Will execute callbacks functions when an event occurs.
	ros::spin();

	// Create a window to display the images received
	if (front_using_helpers) {
		cv::namedWindow(CAMERA1_CV_TOPIC_NAME, CV_WINDOW_KEEPRATIO);
	}
	numFramesWithoutObject = 0;
}

/**
 * @brief Destructor.
 *
 * Releases the memory used by the FrontCVNode.
 *
 */
FrontCVNode::~FrontCVNode() {
	ROS_INFO("%s", (std::string(__PRETTY_FUNCTION__) + ":: destroying FrontCVNode.").c_str());
	if (front_using_helpers) {
		cv::destroyWindow(CAMERA1_CV_TOPIC_NAME);
	}
}

void FrontCVNode::instanciateAllVisibleObjects() {

}

/**
 * @brief Function that is called when an image is received.
 *
 * @param rosMessage The ROS message that contains the image
 *
 */
void FrontCVNode::imageHasBeenReceived(const sensor_msgs::ImageConstPtr& message) {
	std::vector<computer_vision::VisibleObjectData*> messagesToPublish;
	cv_bridge::CvImagePtr pCurrentFrame;
	cv::Mat currentFrame;

	try {
		// Convert sensor_msgs to an opencv image
		pCurrentFrame = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
		currentFrame = pCurrentFrame->image;
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("%s", "A problem occured while trying to convert the image from sensor_msgs::ImageConstPtr to cv:Mat.");
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	try {

		// Loop through the list of visible objects and transmit
		// the received image to each visible object
		for (std::list<VisibleObject*>::iterator it = objectsToSearchFor.begin(); it != objectsToSearchFor.end(); ++it) {
			messagesToPublish = (*it)->retrieveObjectData(currentFrame);
		}

//		ROS_INFO("%s", ("The front cv node received the list of ROS messages that will be sent to the state estimation, which contains " + boost::lexical_cast<std::string>(messagesToPublish.size()) + " element(s).").c_str());

		// Check if no objects were found. If so, only send data if this has been consistent for at least a given amount of frames.
		if (messagesToPublish.size() == 0 || (messagesToPublish.size() != 0 && messagesToPublish[0]->object_type == messagesToPublish[0]->CANNOT_DETERMINE_OBJECT)) {
//			ROS_INFO("%s", "The current message contains no useful information, it will not be sent to the state estimation.");
			numFramesWithoutObject++;
			if (numFramesWithoutObject < FRAME_VISIBILITY_THRESHOLD) return;
		} else {
			numFramesWithoutObject = 0;
		}

		// Publish the VisibleObjectData messages.
		for(std::vector<computer_vision::VisibleObjectData*>::iterator it = messagesToPublish.begin(); it != messagesToPublish.end(); ++it) {
			computer_vision::VisibleObjectData messageToSend;
			messageToSend.object_type = (*it)->object_type;
			messageToSend.pitch_angle = (*it)->pitch_angle;
			messageToSend.yaw_angle = (*it)->yaw_angle;
			messageToSend.x_distance = (*it)->x_distance;
			messageToSend.y_distance = (*it)->y_distance;
			messageToSend.z_distance = (*it)->z_distance;
			frontEndVisibleObjectDataPublisher.publish(messageToSend);
		}

		// Publishes the image after the filters have added information on them.
		cv_bridge::CvImage currentImage;
		currentImage.header.stamp    = ros::Time::now();
		currentImage.encoding        = sensor_msgs::image_encodings::BGR8;
		currentImage.image           = currentFrame;
		frontEndPublisher.publish(currentImage.toImageMsg());

		if (front_using_helpers) {
			// Display the filtered image
			cv::imshow(CAMERA1_CV_TOPIC_NAME, currentFrame);
			cv::waitKey(5);
		}
	} catch (cv::Exception& e) {
		ROS_ERROR("cv::imgshow exception: %s", e.what());
		return;
	}
}
