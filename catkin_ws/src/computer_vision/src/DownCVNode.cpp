#include "DownCVNode.h"

/**
 * @brief Main method used by ROS when the node is launched.
 *
 * @param argc The number of arguments passed when the process is stared.
 * @param argv The arguments passed when the process is started.
 * @return The termination status of the processe's execution.
 */
int main(int argc, char **argv) {

	ros::init(argc, argv, DOWN_CV_NODE_NAME);
	ros::NodeHandle nodeHandle;

	ROS_INFO("Initializing node %s", ros::this_node::getName().c_str());

	std::string imageFeedDown;

	nodeHandle.param<std::string>("image_feed/down", imageFeedDown, "/simulator/camera3/image_raw");

	// Creates a new CVNode object.
	DownCVNode* pDownCVNode = new DownCVNode(nodeHandle, imageFeedDown, DOWN_CV_NODE_RECEPTION_RATE, DOWN_CV_NODE_BUFFER_SIZE);

	// Start receiving images from the camera node (publisher)
	pDownCVNode->receiveImages();

	// Destroy the CVNode object
	delete pDownCVNode;

	// Destroy the node
	ros::shutdown();
}

void DownCVNode::listenToPlanner(planner::CurrentCVTask msg) {
	this->objectsToSearchFor.clear();
	if (msg.currentCVTask == 3) {
		this->objectsToSearchFor.push_back(new Line(*(this)));
	}
}

/**
 * @brief Constructor.
 *
 * Constructs a new DownCVNode object (calls the CVNode constructor).
 *
 */
DownCVNode::DownCVNode(ros::NodeHandle& nodeHandle, std::string topicName, int receptionRate, int bufferSize) : CVNode(nodeHandle, topicName, receptionRate, bufferSize) {
	pLastImage = NULL;

	// Create topic with front end
	this->frontEndPublisher = this->pImageTransport->advertise(CAMERA3_CV_TOPIC_NAME, bufferSize);
	frontEndVisibleObjectDataPublisher = nodeHandle.advertise<computer_vision::VisibleObjectData>(OUTPUT_DATA_TOPIC_NAME, 10);
	
	plannerSubscriber = nodeHandle.subscribe(PLANNER_DATA_DOWN_TOPIC_NAME, 1000, &DownCVNode::listenToPlanner, this);

	std::string currentObject;
	nodeHandle.param<std::string>("cv_down_detect_object", currentObject, "");
	if (!currentObject.empty()) {
		if (currentObject.compare("line") == 0) {
			this->objectsToSearchFor.push_back(new Line(*(this)));
		}
	} else {
		ROS_INFO("%s", "The 'cv_down_detect_object' parameter is empty, the CV will be looking for no object.");
	}

	if (down_using_helpers) {
		cv::namedWindow(CAMERA3_CV_TOPIC_NAME, CV_WINDOW_KEEPRATIO);
	}
}

DownCVNode::~DownCVNode() {
	if (down_using_helpers) {
		cv::destroyWindow(CAMERA3_CV_TOPIC_NAME);
	}
	delete pLastImage;
}

cv::Mat DownCVNode::convertSensorMessageToOpencvImage(const sensor_msgs::ImageConstPtr& sensorMessage) {
	cv_bridge::CvImagePtr cvImagePointer = cv_bridge::toCvCopy(sensorMessage, sensor_msgs::image_encodings::BGR8);
	cv::Mat currentImageMatrix = cvImagePointer->image;
	return currentImageMatrix;
}

/**
 * @brief Function that is called when an image is received.
 *
 * @param rosMessage The ROS message that contains the image
 */
void DownCVNode::imageHasBeenReceived(const sensor_msgs::ImageConstPtr& message) {
	std::vector<computer_vision::VisibleObjectData*> messagesToPublish;
	cv::Mat currentImageMatrix;
	std::list<VisibleObject*>::iterator it;

	try {
		currentImageMatrix = convertSensorMessageToOpencvImage(message);
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	try {
		// Loop through the list of visible objects and transmit
		// the received image to each visible object
		for (it = objectsToSearchFor.begin(); it != objectsToSearchFor.end(); it++) {\
			//FIXME This code will only work if there is ONLY ONE OBJECT in the vector.
			// this is to be improved after the competition so that we can support more than
			// one objects.
			messagesToPublish = (*it)->retrieveObjectData(currentImageMatrix);
		}

		// Check if no objects were found. If so, only send data if this has been consistent for at least a given amount of frames.
		if (messagesToPublish.size() == 0) {
			numFramesWithoutObject++;
			if (numFramesWithoutObject < FRAME_VISIBILITY_THRESHOLD) return;
		} else {
			numFramesWithoutObject = 0;
		}

		for(std::vector<computer_vision::VisibleObjectData*>::iterator it = messagesToPublish.begin(); it !=
		    messagesToPublish.end(); ++it) {
			computer_vision::VisibleObjectData messageToSend;
			messageToSend.object_type = (*it)->object_type;
			messageToSend.pitch_angle = (*it)->pitch_angle;
			messageToSend.yaw_angle = (*it)->yaw_angle;
			messageToSend.x_distance = (*it)->x_distance;
			messageToSend.y_distance = (*it)->y_distance;
			messageToSend.z_distance = (*it)->z_distance;
			frontEndVisibleObjectDataPublisher.publish(messageToSend);
		}


		// Publish the images.
		cv_bridge::CvImage currentImage;
		currentImage.header.stamp = ros::Time::now();
		currentImage.encoding = sensor_msgs::image_encodings::BGR8;
		currentImage.image = currentImageMatrix;
		frontEndPublisher.publish(currentImage.toImageMsg());

		if (down_using_helpers) {
			// Display the filtered image
			cv::imshow(CAMERA3_CV_TOPIC_NAME, currentImageMatrix);
			cv::waitKey(5);
		}
	} catch (cv::Exception& e) {
		ROS_ERROR("cv::imgshow exception: %s", e.what());
		return;
	}
}
