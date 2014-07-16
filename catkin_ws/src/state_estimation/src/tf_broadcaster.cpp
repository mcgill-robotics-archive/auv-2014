#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>

// Custom messages
#include <state_estimation/AUVState.h>
#include <computer_vision/VisibleObjectData.h>

#define PI 3.14159265358979323846

// Object IDs
std::string objectID[] = {
    "gate",
    "buoy",
    "ground_target_1",
    "ground_target_2",
    "ground_target_3",
    "ground_target_4",
    "ground_target_unkown",
    "lane",
    "unknown"
};


// quaternion constructor: x y z w

void broadcastStaticFrames(tf::TransformBroadcaster& broadcaster) {
	broadcaster.sendTransform(
		// Transform data, quaternion for rotations and vector3 for translational vectors
		tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.297, 0.1435, 0.1225)),
			// Give it a time stamp
			ros::Time::now(),
			// from
			"base_link",
			// to
			"/robot/rotation_center"
		)
	);
	
	tf::Quaternion quat;
	// quat.setRPY(-PI/2,0,PI);
	quat.setRPY(-PI/2,0,0);
	broadcaster.sendTransform(
		// Transform data, quaternion for rotations and vector3 for translational vectors
		tf::StampedTransform(
			tf::Transform(quat, tf::Vector3(0.297, 0.1435, 0.1942)), 
			// Give it a time stamp
			ros::Time::now(),
			// from
			"base_link",
			// to
			"/sensors/IMU"
		)
	); //was 1/root 2, 1/root 2, 0, 0
	
	broadcaster.sendTransform(
		// Transform data, quaternion for rotations and vector3 for translational vectors
		tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.594, 0.321, -0.032)),
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
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.594, -0.034, -0.032)),
			// Give it a time stamp
			ros::Time::now(),
			// from
			"base_link",
			// to
			"/sensors/forward_camera_left"
		)
	);

	/*
	broadcaster.sendTransform(
		// Transform data, quaternion for rotations and vector3 for translational vectors
		tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.594, 0.1435, -0.032)),
			// Give it a time stamp
			ros::Time::now(),
			// from
			"base_link",
			// to
			"/sensors/forward_camera_center"
		)
	);
	*/
        
	broadcaster.sendTransform(
		// Transform data, quaternion for rotations and vector3 for translational vectors
		tf::StampedTransform(
			//TODO: CHECK THE NEWLY ADDED ANGLE CHANGE
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.509, 0.1435, 0.245)),
			// Give it a time stamp
			ros::Time::now(),
			// from
			"base_link",
			// to
			"/sensors/downward_camera"
		)
	);
	
	/*	
	// Not measured because grabber is not ready
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
	*/
	/*
	broadcaster.sendTransform(
		// Transform data, quaternion for rotations and vector3 for translational vectors
		tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.499, -0.029, 0.265)),
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
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.499, 0.316, 0.265)),
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
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.696, -0.0245, 0.016)),
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
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.696, 0.3115, 0.016)),
			// Give it a time stamp
			ros::Time::now(),
			// from
			"base_link",
			// to
			"/extremeties/torpedo_right"
		)
	);
	
	// Assume torpedo launcher right now
	broadcaster.sendTransform(
		// Transform data, quaternion for rotations and vector3 for translational vectors
		tf::StampedTransform(
			tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.696, 0.3115, 0.016)),
			// Give it a time stamp
			ros::Time::now(),
			// from
			"base_link",
			// to
			"/extremeties/contact_buoy"
		)
	);
	*/		
}

void cvCallBack(const state_estimation::AUVState::ConstPtr& msg) {
	// Create a transform listener in every callback	
	tf::TransformBroadcaster broadcaster;
	if (msg->hasTarget) {
		// Visible Object Data
		computer_vision::VisibleObjectData object = msg->visibleObjectData;
		
		tf::Quaternion quat = tf::createQuaternionFromRPY(0.0, object.pitch_angle, object.yaw_angle);
		tf::Vector3 vect(object.x_distance, object.y_distance, object.z_distance);
		
		if (object.object_type != 255) {
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
					"/sensors/downward_camera",
					// to
					refFrame
				)
			);
		}
	}

	broadcastStaticFrames(broadcaster);
}

void imuCallBack(const geometry_msgs::PoseStamped::ConstPtr& msg) {
	tf::TransformBroadcaster broadcaster;
	
	tf::Quaternion orientation = tf::Quaternion(msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);

	broadcaster.sendTransform
	(
		// Transform data, quaternion for rotations and vector3 for translational vectors
		tf::StampedTransform
		(
			tf::Transform
			(
				orientation, 
				tf::Vector3(0.0, 0.0, 0.0)
			),
			ros::Time::now(), // Give it a time stamp
			// from
				"/sensors/IMU",
			// to
				"/sensors/IMU_global_reference"
		)
	);

	/*
	Define vectors in the global frame (IMU_global_reference)
	


	*/
	
	// tf::TransformListener tf_listener;

	// tf::Stamped<tf::Vector3> b_global = tf::Stamped<tf::Vector3>(tf::Vector3(0,0,1),msg->header.stamp,"/sensors/IMU_global_reference");
	// tf::Stamped<tf::Vector3> a_body = tf::Stamped<tf::Vector3>(tf::Vector3(0,0,1),msg->header.stamp,"/robot/rotation_center");
	// tf::Stamped<tf::Vector3> a_global;	

	// tf_listener.waitForTransform("/sensors/IMU_global_reference", "/robot/rotation_center", ros::Time(0), ros::Duration(0.4));
	// tf_listener.transformVector("/sensors/IMU_global_reference", a_body, a_global); 

	// tf::Vector3 axis = tf::tfCross(a_global,b_global);
	// tf::Quaternion quat = tf::Quaternion(axis,tf::tfAngle(a_global,b_global));



	
	
	//double roll,pitch,yaw;
	//tf::Matrix3x3(orientation).getRPY(roll,pitch,yaw);
	//tf::Quaternion yawOnlyQuat;
	//yawOnlyQuat.setRPY(0,0,pitch); //lol this is on purpose (particular to mounting of IMU in July 2014)
	//yawOnlyQuat.inverse()

	// broadcaster.sendTransform
	// (
	// 	// Transform data, quaternion for rotations and vector3 for translational vectors
	// 	tf::StampedTransform
	// 	(
	// 		tf::Transform
	// 		(
	// 			quat, 
	// 			tf::Vector3(0.0, 0.0, 0.0)
	// 		),
	// 		ros::Time::now(), // Give it a time stamp
	// 		// from
	// 			"/robot/rotation_center",
	// 		// to
	// 			"/robot/horizon"
	// 	)
	// );
	

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "tf_broadcaster");

	ros::NodeHandle n;
	tf::TransformBroadcaster broadcaster;

	ros::Subscriber cvSub = n.subscribe("state_estimation/state_estimate", 1000, cvCallBack);
	ros::Subscriber imuSub = n.subscribe("state_estimation/pose", 1000, imuCallBack);

	ros::spin();

	return 0;
}
