#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <tf/transform_listener.h>

#include "tf_utilities/transform.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "sample_client");
	ros::NodeHandle n;

	ros::ServiceClient client = n.serviceClient<tf_utilities::transform>("tf_utilities/transform_pose");

	tf_utilities::transform srv;

	// Just use a dummy data
	// Here, we want to transform the pose from "base_footprint" to "base_link"
	geometry_msgs::PoseStamped testPose;
	testPose.header.frame_id = "base_footprint";
	testPose.header.stamp = ros::Time::now();
	testPose.header.seq = 0;
	testPose.pose.position.x = 1.0;
	testPose.pose.position.y = 1.0;
	testPose.pose.position.z = 1.0;

	srv.request.pose = testPose;
	srv.request.target_frame = "base_link";

	if(client.call(srv)) {
		// Now the data is stored in srv.response.trans_pose
	} else {
		ROS_ERROR("ERROR!");

		return 1;
	}

	return 0;
}