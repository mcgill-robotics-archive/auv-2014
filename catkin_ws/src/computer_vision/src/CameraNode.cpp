#include "CameraNode.h"

int main(int argc, char **argv) {

	if (argc == 3) {
		// Initialize the node
		ros::init(argc, argv, argv[1]);
		ros::NodeHandle nodeHandle;

		// Set the topic name to: [node name]_feed
		char topicName[strlen(argv[1]) + 5];
		strcpy(topicName, argv[1]);
		strcat(topicName, "_feed");

		// Create a new CameraNode object
		CameraNode* pCameraNode = new CameraNode(nodeHandle, topicName, argv[2], 10);

		// Start sending images to computer vision node (subscriber)
		pCameraNode->sendImages();

		// Destroy the CameraNode object
		delete pCameraNode;

		// Destroy the node
		ros::shutdown();
	}
	else
		std::cout << "Usage: rosrun computer_vision CVNode [node name] [video source]" << std::endl;
}

/**
 * Constructs a CameraNode object. The camera node is used to send images
 * to the computer vision node.
 * @param nodeHandle The node handle
 * @param topicName The name of the topic on which the images will published
 * @param captureSource The path to the video file to be transmitted (if empty, the default camera is used)
 * @param transmissionRate The rate at which the images are published on the topic
 */
CameraNode::CameraNode(ros::NodeHandle& nodeHandle, const char* topicName, const char* captureSource, int transmissionRate) {
	this->pImageTransport = new image_transport::ImageTransport(nodeHandle);
	this->pCamera = new Camera(captureSource);
	this->pLastImage = new cv_bridge::CvImage();
	this->transmissionRate = transmissionRate;
	this->publisher = pImageTransport->advertise(topicName, 1);
}

/**
 * Releases the memory used by the CameraNode object.
 */
CameraNode::~CameraNode() {
	this->publisher.shutdown();
	delete this->pImageTransport;
	delete this->pCamera;
	delete this->pLastImage;
}

/**
 * Takes a frame from the camera and sends it to the computer vision node.
 */
void CameraNode::sendImages() {
	ros::Rate loop_rate(transmissionRate);
	cv::Mat* pCurrentFrame;

	while (ros::ok()) {

		try {
			// Read frame from camera
			pCurrentFrame = pCamera->captureFrame();
		}
		catch (cv::Exception& e) {
			ROS_ERROR("opencv exception: %s", e.what());
			return;
		}

		// Convert camera frame to a cv_bridge::CvImage
		toCvImage(pCurrentFrame);

		try {
			// Convert cv_bridge::CvImage image to ROS image message and publish
			publisher.publish(pLastImage->toImageMsg());
		}
		catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}

/**
 * Helper function used to convert an opencv image to cv_bridge image.
 * @param pFrame Pointer to the opencv image to convert
 */
void CameraNode::toCvImage(cv::Mat* pFrame) {

	// Delete last image and create a new one
	delete pLastImage;
	pLastImage = new cv_bridge::CvImage();

	// Set image properties
	pLastImage->header.stamp    = ros::Time::now();
	pLastImage->encoding        = sensor_msgs::image_encodings::BGR8;
	pLastImage->image           = *pFrame;
}

