/**
 * @file CVTestNode.cpp
 * @author Jean-Sebastien Dery
 * @date January 11th 2014
 * @brief This is a ROS node used to test basic things for the computer vision.
 */
#include "CVTestNode.h"
#include "computer_vision/VisibleObjectData.h"

/**
 * Defines the number of seconds between every delays when the node is waiting for someone to publish the topic.
 */
#define DELAY_BETWEEN_INFOS 5

/**
 * @brief Main method used by ROS when the node is launched.
 *
 * @param argc The number of arguments passed when the process is stared.
 * @param argv The arguments passed when the process is started.
 * @return The termination status of the processe's execution.
 */
int main(int argc, char **argv) {
	// Initializes the ROS node.
	ros::init(argc, argv, "cv_test_node");

	ROS_INFO("%s", "Starting CVTestNode.");

	// Create the node handle that will be used in the communication.
	ros::NodeHandle nodeHandle;

	// Creates the publisher for the forward cameras.
	ros::Publisher forwardCamerasPublisher = nodeHandle.advertise<computer_vision::VisibleObjectData>("forwardCameras", 1000);

	ros::Rate loop_rate(10);

	while (ros::ok()) {

		computer_vision::VisibleObjectData visibleObjectData;
		visibleObjectData.object_type = visibleObjectData.DOOR;
		visibleObjectData.pitch_angle = 90.0;
		visibleObjectData.yaw_angle = -90.0;
		visibleObjectData.x_distance = 1;
		visibleObjectData.y_distance = 2;
		visibleObjectData.z_distance = 3;

		forwardCamerasPublisher.publish(visibleObjectData);

		ros::spinOnce();

		loop_rate.sleep();
	}

	ROS_INFO("%s", "Terminating CVTestNode.");

	// Destroy the node
	ros::shutdown();
}
