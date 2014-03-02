/*
 * 	Dynamic broadcaster that subscribes to position information and broadcast corresponding transforms
*/
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_broadcaster.h"

tf::TransformBroadcaster broadcaster;

void callBack(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	broadcaster.sendTransform(
		// Transform data, quaternion for rotations and vector3 for translational vectors
		tf::StampedTransform(
			tf::Transform(
				tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w), 
				tf::Vector3(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z)
			),
			// Give it a time stamp
			ros::Time::now(),
			// from
			"base_link",
			// to
			"base_laser"
		)
	);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "dynamic_broadcaster");
	
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("/sample_topic", 1000, callBack);
	
	ros::spin();
}
