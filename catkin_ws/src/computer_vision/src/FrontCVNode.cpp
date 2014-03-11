#include "FrontCVNode.h"
#include "FrontCameraNode.h"


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

	ROS_INFO("Initializing node %s", ros::this_node::getName().c_str());

	// Creates a new CVNode object.
	FrontCVNode* pFrontCVNode = new FrontCVNode(nodeHandle, FRONT_CAMERA_NODE_TOPIC, FRONT_CV_NODE_RECEPTION_RATE, FRONT_CV_NODE_BUFFER_SIZE);

	// Start receiving images from the camera node (publisher)
	pFrontCVNode->receiveImages();

	// Destroy the CVNode object
	delete pFrontCVNode;

	// Destroy the node
	ros::shutdown();
}

/**
 * @brief Constructor.
 *
 * Constructs a new FrontCVNode object (calls the CVNode constructor).
 *
 */
FrontCVNode::FrontCVNode(ros::NodeHandle& nodeHandle, std::string topicName, int receptionRate, int bufferSize) : CVNode(nodeHandle, topicName, receptionRate, bufferSize) {
	pLastImageLeftCamera = NULL;
	pLastImageRightCamera = NULL;

	// Create topics with front end
	frontEndPublisher = pImageTransport->advertise(CAMERA1_CV_TOPIC_NAME, bufferSize);
	frontEndVisibleObjectDataPublisher = nodeHandle.advertise<computer_vision::VisibleObjectData>(OUTPUT_DATA_TOPIC_NAME, 10);

	// Construct the list of VisibleObjects
	this->visibleObjectList.push_back(new Gate());
	//this->visibleObjectList.push_back(new Buoy());
	// Create a window to display the images received
	cv::namedWindow(FRONT_CAMERA_NODE_TOPIC, CV_WINDOW_KEEPRATIO);
}

/**
 * @brief Destructor.
 *
 * Releases the memory used by the FrontCVNode.
 *
 */
FrontCVNode::~FrontCVNode() {
	cv::destroyWindow(FRONT_CAMERA_NODE_TOPIC);

	// Releases the memory used by the cv::Mat object for the image.
	delete pLastImageLeftCamera;
	delete pLastImageRightCamera;
}


/**
 * @brief Function that is called when an image is received.
 *
 * @param rosMessage The ROS message that contains the image
 *
 */
void FrontCVNode::receiveImage(const sensor_msgs::ImageConstPtr& message) {
	std::vector<computer_vision::VisibleObjectData*> messagesToPublish;
	cv_bridge::CvImagePtr pCurrentFrame;
	cv::Mat currentFrame;

	try {
		// Convert sensor_msgs to an opencv image
		pCurrentFrame = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
		currentFrame = pCurrentFrame->image;
	} catch (cv_bridge::Exception& e) {
		ROS_ERROR("$s", "A problem occured while trying to convert the image from sensor_msgs::ImageConstPtr to cv:Mat.");
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	try {

		// Loop through the list of visible objects and transmit
		// the received image to each visible object
		for (std::list<VisibleObject*>::iterator it = visibleObjectList.begin(); it != visibleObjectList.end(); ++it) {
			messagesToPublish = (*it)->retrieveObjectData(currentFrame);
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

		// Display the filtered image
		cv::imshow(FRONT_CAMERA_NODE_TOPIC, currentFrame);
		cv::waitKey(5);
	} catch (cv::Exception& e) {
		ROS_ERROR("cv::imgshow exception: %s", e.what());
		return;
	}
}

///**
// * Converts a sensor_msgs::ImageConstPtr to a cv::Mat object that OpenCV can use to run the filters.
// *
// * @param message The sensor_msgs::ImageConstPtr to be converted.
// * @return The cv:Mat object that can be used by OpenCV.
// */
//cv::Mat convertFromSensorToOpenCV(const sensor_msgs::ImageConstPtr& message) {
//	cv_bridge::CvImagePtr pCurrentFrame;
//
//	try {
//		// Convert sensor_msgs to an opencv image
//		pCurrentFrame = cv_bridge::toCvCopy(message, sensor_msgs::image_encodings::BGR8);
//		return (pCurrentFrame->image);
//	} catch (cv_bridge::Exception& e) {
//		ROS_ERROR("$s", "A problem occured while trying to convert the image from sensor_msgs::ImageConstPtr to cv:Mat.");
//		ROS_ERROR("cv_bridge exception: %s", e.what());
//		// Returns an empty cv::Mat object.
//		return (cv::Mat());
//	}
//}
