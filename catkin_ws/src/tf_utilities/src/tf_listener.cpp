#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

void transformPoint(const tf::TransformListener& listener) {
	// Point to be transformed in current frame
	geometry_msgs::PointStamped point_to_be_tranformed;
	
	point_to_be_tranformed.header.frame_id = "current_frame";
	// Use the most recent transform available
	point_to_be_tranformed.header.stamp = ros::Time();
	
	// Assgin random positions to point as a demo
	point_to_be_tranformed.point.x = 1.0;
	point_to_be_tranformed.point.y = 2.0;
	point_to_be_tranformed.point.z = 3.0;
	
	try {
		// Point transformed into target frame
		geometry_msgs::PointStamped point_transformed;
		listener.transformPoint("target_frame", point_to_be_tranformed, point_transformed);
		
		// Yay! We got the point and can do whatever!
		// Point information is stored in point_transformed
	} catch (tf::TransformException& ex) {
		// Exception occurs
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "tf_listener");
	ros::NodeHandle n;
	
	tf::TransformListener listener(ros::Duration(10));
	
	// Transform a point every second. Again for demo purposes
	// We can literally call transform at any time
	ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));
	
	ros::spin();
}
