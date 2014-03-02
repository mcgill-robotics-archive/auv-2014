#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
	ros::init(argc, argv, "tf_publisher");
	ros::NodeHandle n;
	
	ros::Rate r(100);
	tf::TransformBroadcaster broadcaster;
	
	while(n.ok()) {
		broadcaster.sendTransform(
			// Transform data, quaternion for rotations and vector3 for translational vectors
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, 0.0)),
				// Give it a time stamp
				ros::Time::now(),
				// from
				"base_link",
				// to
				"base_laser"
			)
		);
		
		r.sleep();
	}
}
