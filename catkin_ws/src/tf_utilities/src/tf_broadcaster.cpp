#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

// Make broadcaster global
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
			// refrence frame does not have a name now
			"reference_frame",
			// to
			"/sensor/forward_camera_center"
		)
	);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "tf_publisher");
	ros::NodeHandle n;
	
	ros::Rate r(100);

	ros::Subscriber sub = n.subscribe("/sample_topic", 1000, callBack);
	
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
				"/sensors/IMU"
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
				"/sensors/forward_camera_right"
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
				"/sensors/forward_camera_left"
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
				"/sensors/forward_camera_center"
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
				"/sensors/downward_camera"
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
				"/extremeties/grabber"
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
				"/extremeties/marker_left"
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
				"/extremeties/marker_right"
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
				"/exteremeties/torpedo_left"
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
				"/extremeties/torpedo_right"
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
				"/extremeties/contact_bouy"
			)
		);		
		
		
		r.sleep();
	}
}
