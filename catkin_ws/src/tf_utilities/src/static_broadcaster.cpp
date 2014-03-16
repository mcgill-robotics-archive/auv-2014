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
				tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1.0, 0.5, 0.5)),
				// Give it a time stamp
				ros::Time::now(),
				// from
				"base_link",
				// to
				"IMU"
			)
		);
		
		broadcaster.sendTransform(
			// Transform data, quaternion for rotations and vector3 for translational vectors
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3( 2.0, 1.0, 1.0)),
				// Give it a time stamp
				ros::Time::now(),
				// from
				"base_link",
				// to
				"camera_right"
			)
		);
		
		broadcaster.sendTransform(
			// Transform data, quaternion for rotations and vector3 for translational vectors
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(2.0, 0.0, 1.0)),
				// Give it a time stamp
				ros::Time::now(),
				// from
				"base_link",
				// to
				"camera_left"
			)
		);
		
		broadcaster.sendTransform(
			// Transform data, quaternion for rotations and vector3 for translational vectors
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(2.0, 0.5, 1.0)),
				// Give it a time stamp
				ros::Time::now(),
				// from
				"base_link",
				// to
				"camera_down"
			)
		);
		
		broadcaster.sendTransform(
			// Transform data, quaternion for rotations and vector3 for translational vectors
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1.7, 0.5, 1.0)),
				// Give it a time stamp
				ros::Time::now(),
				// from
				"base_link",
				// to
				"grabber"
			)
		);
		
		broadcaster.sendTransform(
			// Transform data, quaternion for rotations and vector3 for translational vectors
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.7, 0.0, 1.0)),
				// Give it a time stamp
				ros::Time::now(),
				// from
				"base_link",
				// to
				"marker_left"
			)
		);
		
		broadcaster.sendTransform(
			// Transform data, quaternion for rotations and vector3 for translational vectors
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.7, 1.0, 1.0)),
				// Give it a time stamp
				ros::Time::now(),
				// from
				"base_link",
				// to
				"marker_right"
			)
		);
			
		broadcaster.sendTransform(
			// Transform data, quaternion for rotations and vector3 for translational vectors
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(2.0, 0.0, 0.5)),
				// Give it a time stamp
				ros::Time::now(),
				// from
				"base_link",
				// to
				"torpedo_left"
			)
		);
		
		broadcaster.sendTransform(
			// Transform data, quaternion for rotations and vector3 for translational vectors
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(2.0, 1.0, 0.5)),
				// Give it a time stamp
				ros::Time::now(),
				// from
				"base_link",
				// to
				"torpedo_right"
			)
		);
		
		broadcaster.sendTransform(
			// Transform data, quaternion for rotations and vector3 for translational vectors
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(2.0, 0.5, 0.5)),
				// Give it a time stamp
				ros::Time::now(),
				// from
				"base_link",
				// to
				"contact_bouy"
			)
		);
		
		broadcaster.sendTransform(
			// Transform data, quaternion for rotations and vector3 for translational vectors
			tf::StampedTransform(
				tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(2.0, 0.5, 0.5)),
				// Give it a time stamp
				ros::Time::now(),
				// from
				"base_link",
				// to
				"contact_bouy"
			)
		);		
		
		
		r.sleep();
	}
}
