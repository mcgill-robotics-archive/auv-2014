#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>
#include <tf/transform_listener.h>

#include "tf_utilities/transform.h"

bool transform(tf_utilities::transform::Request&, tf_utilities::transform::Response&);
geometry_msgs::PoseStamped& transformPose(const tf::TransformListener&, const geometry_msgs::PoseStamped&, std::string target_frame);

tf::TransformListener listener(ros::Duration(10));
geometry_msgs::PoseStamped transformedPose;

bool transform(tf_utilities::transform::Request& req,
				tf_utilities::transform::Response& res) {
	res.trans_pose = transformPose(listener, req.pose, req.target_frame);
	// For debugging
	ROS_INFO("Transformed");
}

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
