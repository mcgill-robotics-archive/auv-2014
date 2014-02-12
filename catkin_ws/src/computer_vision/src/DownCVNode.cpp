#include "DownCVNode.h"
#include "DownCameraNode.h"

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

	// Creates a new CVNode object.
	DownCVNode* pDownCVNode = new DownCVNode(nodeHandle, DOWN_CAMERA_NODE_TOPIC, DOWN_CV_NODE_RECEPTION_RATE, DOWN_CV_NODE_BUFFER_SIZE);

	// Start receiving images from the camera node (publisher)
	pDownCVNode->receiveImages();

	// Destroy the CVNode object
	delete pDownCVNode;

	// Destroy the node
	ros::shutdown();
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
	frontEndVisibleObjectDataPublisher = nodeHandle.advertise<computer_vision::VisibleObjectData>(DATA_TOPIC_NAME, 10);
	
	//this->visibleObjectList.push_back(new MarkerTarget());

	// FIXME: There is a problem with LineTarget::retrieveObjectData()
	this->visibleObjectList.push_back(new LineTarget());

	cv::namedWindow(DOWN_CAMERA_NODE_TOPIC, CV_WINDOW_KEEPRATIO);
}

DownCVNode::~DownCVNode() {
	cv::destroyWindow(DOWN_CAMERA_NODE_TOPIC);

	delete pLastImage;
}


/**
 * @brief Function that is called when an image is received.
 *
 * @param rosMessage The ROS message that contains the image
 */
void DownCVNode::receiveImage(const sensor_msgs::ImageConstPtr& message) {
	std::vector<computer_vision::VisibleObjectData*> messagesToPublish;
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
		for (it = visibleObjectList.begin(); it != visibleObjectList.end(); it++) {
			messagesToPublish = (*it)->retrieveObjectData(currentFrame);
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
		}


		// Publish the images.
		cv_bridge::CvImage currentImage;
		currentImage.header.stamp = ros::Time::now();
		currentImage.encoding = sensor_msgs::image_encodings::BGR8;
		currentImage.image = currentFrame;
		frontEndPublisher.publish(currentImage.toImageMsg());

		// Display the filtered image
		cv::imshow(DOWN_CAMERA_NODE_TOPIC, currentFrame);
		cv::waitKey(5);
	} catch (cv::Exception& e) {
		ROS_ERROR("cv::imgshow exception: %s", e.what());
		return;
	}
}
