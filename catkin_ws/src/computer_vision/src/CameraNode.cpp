#include "CameraNode.h"

int main(int argc, char **argv) {
	// Initialize the node
    ros::init(argc, argv, "camera_node");
    ros::NodeHandle nodeHandle;
    
    // Create a new CameraNode object
    //CameraNode* pCameraNode = new CameraNode(nodeHandle, "camera_feed", argv[1], 10);

	// This is for Gazebo.
	CameraNode* pCameraNode = new CameraNode(nodeHandle, "camera_feed", argv[1], 10);

    // Start sending images to computer vision node (subscriber)
    pCameraNode->sendImage();

    // Destroy the CameraNode object
    delete pCameraNode;

    // Destroy the node
    ros::shutdown();
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
void CameraNode::sendImage() {
	ros::Rate loop_rate(transmissionRate);
	cv::Mat* pFrame;

	while (ros::ok()) {

		try {
			// Read frame from camera
			pFrame = pCamera->captureFrame();
		}
		catch (cv::Exception& e) {
			ROS_ERROR("opencv exception: %s", e.what());
			return;
		}

		// Convert camera frame to a cv_bridge image
		encodeFrame(pFrame);

		try {
			// Convert cv_bridge image to ROS image message and publish
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
void CameraNode::encodeFrame(cv::Mat* pFrame) {

	// Delete last image and create a new one
	delete pLastImage;
	pLastImage = new cv_bridge::CvImage();

	// Set image properties
	pLastImage->header.stamp    = ros::Time::now();
	pLastImage->encoding        = sensor_msgs::image_encodings::BGR8;
	pLastImage->image           = *pFrame;
}

