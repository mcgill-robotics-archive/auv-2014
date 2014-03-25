#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>

// Custom messages
#include <state_estimation/AUVState.h>
#include <computer_vision/VisibleObjectData.h>

// Object IDs
std::string objectID[] = {
    "door",
    "buoy",
    "ground_target_1",
    "ground_target_2",
    "ground_target_3",
    "ground_target_4",
    "ground_target_unkown",
    "lane",
    "unknown"
};

void broadcastStaticFrames(tf::TransformBroadcaster& broadcaster) {
	broadcaster.sendTransform(
		// Transform data, quaternion for rotations and vector3 for translational vectors
		tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.5, 0.5, 0.5)),
			// Give it a time stamp
			ros::Time::now(),
			// from
			"base_link",
			// to
			"/robot/rotation_center"
		)
	);

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
}

void callBack(const state_estimation::AUVState::ConstPtr& msg) {
	// Create a transform listener in every callback	
	tf::TransformBroadcaster broadcaster;
	
	// Visible Object Data
	computer_vision::VisibleObjectData object = msg->visibleObjectData;
	
	tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, object.pitch_angle, object.yaw_angle);
	tf::Vector3 vect(object.x_distance, object.y_distance, object.z_distance);
	
	if (object.object_type == 255) {
		// If objectID is 255 we did not see anything
		// So we dont publish anything
		std::string refFrame = "/target/" + objectID[object.object_type];
		
		broadcaster.sendTransform(
			// Transform data, quaternion for rotations and vector3 for translational vectors
			tf::StampedTransform(
				tf::Transform(
					quat, 
					vect
				),
				// Give it a time stamp
				ros::Time::now(),
				// refrence frame does not have a name now
				"/sensors/forward_camera_center",
				// to
				refFrame
			)
		);
	}

	broadcastStaticFrames(broadcaster);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "tf_broadcaster");

	ros::NodeHandle n;
	tf::TransformBroadcaster broadcaster;

	ros::Subscriber sub = n.subscribe("state_estimation/state_estimate", 1000, callBack);

	ros::spin();

	return 0;
}
