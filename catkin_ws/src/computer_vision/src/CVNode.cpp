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
 * @brief Constructor.
 *
 * Constructs a new CVNode object. The computer vision node receives
 * images from the camera node and transfers them to each VisibleObject.
 *
 * @param nodeHandle The ROS node handle
 * @param topicName The name of the topic on which the images are published
 */
CVNode::CVNode(ros::NodeHandle& nodeHandle, std::string topicName, int receptionRate, int bufferSize) {
	ROS_INFO("%s", (std::string(__PRETTY_FUNCTION__) + ": initializing CVNode (the parent of FrontCVNod and DownCVNode).").c_str());

	this->receptionRate = receptionRate;
	this->pImageTransport = new image_transport::ImageTransport(nodeHandle);

	// Instanciate the subscribers
	ROS_INFO("%s", (std::string(__PRETTY_FUNCTION__) + ": will subscribe to the topic named '" + topicName + "'.").c_str());
	cameraNodeSubscriber = pImageTransport->subscribe(topicName, bufferSize, &CVNode::receiveImage, this);

	// Wait for publisher(s) to be ready
	while (cameraNodeSubscriber.getNumPublishers() == 0) {
		ROS_INFO_THROTTLE(DELAY_BETWEEN_INFOS, (std::string(__PRETTY_FUNCTION__) + ": Waiting for a publisher to start publishing on the topic '" + topicName + "'.").c_str());
	}

	nodeHandle.param<bool>("cv_front_using_helper_windows", front_using_helpers, 0);
	nodeHandle.param<bool>("cv_down_using_helper_windows", down_using_helpers, 0);
	nodeHandle.param<int>("start_hsv_hue_threshold", start_hsv_hue_thresh, 0);
	nodeHandle.param<int>("end_hsv_hue_threshold", end_hsv_hue_thresh, 0);
	nodeHandle.param<int>("start_hsv_value_threshold", start_hsv_value_thresh, 0);
	nodeHandle.param<int>("end_hsv_value_threshold", end_hsv_value_thresh, 0);
	nodeHandle.param<double>("camera_focal_length", camera_focal_length, 0);
	nodeHandle.param<double>("camera_sensor_height", camera_sensor_height, 0);

	//Gate parameters
	nodeHandle.param<int>("gate_min_number_points_contour", gp.min_number_points_contour, 0);
	nodeHandle.param<int>("gate_pole_desired_angle_deg", gp.pole_desired_angle_deg, 0);
	nodeHandle.param<int>("gate_pole_angle_error_deg", gp.pole_angle_error_deg, 0);
	ROS_INFO("%s", "DESIRED ANGLE: ");	
	ROS_INFO("%d", gp.pole_desired_angle_deg);
	nodeHandle.param<int>("gate_min_pole_ratio", gp.min_pole_ratio, 0);
	nodeHandle.param<int>("gate_height_m", gp.gate_height_m, 0);
	nodeHandle.param<int>("gate_width_m", gp.gate_width_m, 0);

	ROS_INFO("%s", (std::string(__PRETTY_FUNCTION__) + ": CVNode (the parent of FrontCVNode and DownCVNode) has been initialized.").c_str());
}

/**
 * @brief Destructor.
 *
 * Releases the memory used by the CVNode object.
 */
CVNode::~CVNode() {
	// Delete all VisibleObjects
	while (!visibleObjectList.empty()) {
		delete visibleObjectList.front();
		visibleObjectList.pop_front();
	}

	frontEndPublisher.shutdown();

	delete pImageTransport;
}

/**
 * @brief Function that dictates the rate at which the node checks if an image is received.
 */
void CVNode::receiveImages() {
	ROS_INFO("%s", (std::string(__PRETTY_FUNCTION__) + ":: will start to receive images from publisher.").c_str());

	ros::Rate loop_rate(receptionRate);

	while (ros::ok()) {
		// Check if the camera is still publishing
		if (cameraNodeSubscriber.getNumPublishers() == 0) {
			break;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}
