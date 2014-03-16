#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <tf/transform_listener.h>

#include "tf_utilities/transform.h"

bool transform(tf_utilities::transform::Request&, tf_utilities::transform::Response&);
geometry_msgs::PoseStamped& transformPose(const tf::TransformListener&, const geometry_msgs::PoseStamped&, std::string target_frame);

// Global variables
tf::TransformListener listener(ros::Duration(10));
geometry_msgs::PoseStamped transformedPose;

// The service routine call
// Takes the pose in current frame and target frame
// Gives the transformed pose in target frame
bool transform(tf_utilities::transform::Request& req,
				tf_utilities::transform::Response& res) {
	res.trans_pose = transformPose(listener, req.pose, req.target_frame);
	// For debugging
	ROS_INFO("Transformed");

	return true;
}

// Use the tf listener to transform the pose
geometry_msgs::PoseStamped& transformPose(const tf::TransformListener& listener, 
					const geometry_msgs::PoseStamped& pose, std::string target_frame) {
	
	try {
		listener.transformPose(target_frame, pose, transformedPose);
		
		// Yay! We got the point and can do whatever!
		return transformedPose;
	} catch (tf::TransformException& ex) {
		// Exception occurs
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "tf_service");
	ros::NodeHandle n;
	
	ros::ServiceServer service = n.advertiseService("tf_utilities/transform_pose", transform);
	
	ros::spin();
	
	return 0;
}
