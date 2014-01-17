#include "FrontCVNode.h"

/**
 * Defines the name this node will have during its execution.
 */
const std::string ROS_NODE_NAME = "front_cv_node";

/**
 * Defines what will be the reception rate of
 */
const int RECEPTION_RATE = 1;

/**
 * The topic name used for the front_cv_node to publish the VisibleObjectData.
 */
const std::string DATA_TOPIC_NAME = "front_cv_data";


ros::Publisher visibleObjectDataPublisher;
ros::Publisher frontCVCamera1Publisher;
ros::Publisher frontCVCamera2Publisher;

/**
 * The topic name used for the front_cv_node to publish the cv::Mat object after the filters have been applied on camera1.
 */
const std::string CAMERA1_CV_TOPIC_NAME = "front_cv_camera1";

/**
 * @brief Main method used by ROS when the node is launched.
 *
 * @param argc The number of arguments passed when the process is stared.
 * @param argv The arguments passed when the process is started.
 * @return The termination status of the processe's execution.
 */
int main(int argc, char **argv) {

	if (argc > 1) {
		ros::init(argc, argv, ROS_NODE_NAME);
		ros::NodeHandle nodeHandle;

		// TODO: I put the publishers' initialization there, but it could be moved somewhere else.
		visibleObjectDataPublisher = nodeHandle.advertise<computer_vision::VisibleObjectData>(DATA_TOPIC_NAME, 10);
		frontCVCamera1Publisher = nodeHandle.advertise<computer_vision::VisibleObjectData>(CAMERA1_CV_TOPIC_NAME, 10);

		ROS_INFO("%s", ("Initializing the node " + ros::this_node::getName() + ".").c_str());

		// Create the list of topics to listen to
		std::list<std::string> topicList;
		for (int i = 1; i < argc; i++) {
			std::string topic = std::string(argv[i]);
			ROS_INFO("%s", (ros::this_node::getName() + " will be listening to the topic named: " + topic).c_str());
			topicList.push_back(topic);
		}

		// Creates a new CVNode object.
		FrontCVNode* pFrontCVNode = new FrontCVNode(nodeHandle, topicList, RECEPTION_RATE);

		// Start receiving images from the camera node (publisher)
		pFrontCVNode->receiveImages();

		// Destroy the CVNode object
		delete pFrontCVNode;

		// Destroy the node
		ros::shutdown();
	} else {
		ROS_ERROR("%s", "The topics that the node should subscribe to are not present.");
		ROS_ERROR("%s", "Usage: rosrun computer_vision FrontCVNode [topic 1] [topic 2] ... [topic n]");
	}
}

/**
 * @brief Constructor.
 *
 * Constructs a new FrontCVNode object (calls the CVNode constructor).
 *
 */
FrontCVNode::FrontCVNode(ros::NodeHandle& nodeHandle, std::list<std::string> topicList, int receptionRate) : CVNode(nodeHandle, topicList, receptionRate) {
	// Create topic with front end
	this->publisher = this->pImageTransport->advertise(CAMERA1_CV_TOPIC_NAME, 1);

	// Construct the list of VisibleObjects
	visibleObjects.push_back(new Door());
}


/**
 * @brief Function that is called when an image is received.
 *
 * @param rosMessage The ROS message that contains the image
 */
void FrontCVNode::receiveImage(const sensor_msgs::ImageConstPtr& message, const std::string &topicName) {
	std::vector<computer_vision::VisibleObjectData*> messagesToPublish;

//	cv::Mat currentFrame = convertFromSensorToOpenCV(message);
	cv::Mat currentFrame;
	cv_bridge::CvImagePtr pCurrentFrame;

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
		for (std::list<VisibleObject*>::iterator it = visibleObjects.begin(); it != visibleObjects.end(); ++it) {
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
			visibleObjectDataPublisher.publish(messageToSend);
		}

		// Publish the images.
		cv_bridge::CvImage currentImage;
		currentImage.header.stamp    = ros::Time::now();
		currentImage.encoding        = sensor_msgs::image_encodings::BGR8;
		currentImage.image           = currentFrame;
		publisher.publish(currentImage.toImageMsg());

		// Display the filtered image
		cv::imshow(topicName, currentFrame);
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
