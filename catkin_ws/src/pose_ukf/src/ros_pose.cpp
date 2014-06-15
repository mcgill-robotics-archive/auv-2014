/* Ros node for pose estimation algorithm
 *
 * Author: Max Krogius
 */

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Imu.h"

#include "pose_ukf.h"

ros::Publisher pub;
ros::Subscriber sub;
ukf estimator(3);
double acc[3], gyro[3], quaternion[4];

void vectorToArray(double array[3], geometry_msgs::Vector3 vector) {
	array[0] = vector.x;
	array[1] = vector.y;
	array[2] = vector.z;
}

void arrayToQuaternion(geometry_msgs::Quaternion* quaternion, double array[4]) {
	quaternion->w = array[0];
	quaternion->x = array[1];
	quaternion->y = array[2];
	quaternion->z = array[3];
}

void dataCallback(const sensor_msgs::Imu::ConstPtr& imu) {
	vectorToArray(acc, imu->linear_acceleration);
	vectorToArray(gyro, imu->angular_velocity);

	double accNorm = sqrt(acc[0]*acc[0]+acc[1]*acc[1]+acc[2]*acc[2]);
	if (accNorm == 0) accNorm = 1;
	for (int i = 0; i < 3; i++)
	{
		gyro[i] *= 0.026;
		acc[i] *= -9.8/accNorm;
	}


	estimator.update(acc, gyro, quaternion);
	geometry_msgs::Quaternion quat = geometry_msgs::Quaternion();
	arrayToQuaternion(&quat, quaternion);

	geometry_msgs::PoseStamped posStamped = geometry_msgs::PoseStamped();
	posStamped.header = imu->header;
	posStamped.header.frame_id = "base_footprint";
	posStamped.pose.orientation = quat;

	pub.publish(posStamped);
}


int main (int argc, char **argv) {
	ros::init(argc, argv, "pose_ukf");
	ros::NodeHandle node;

	pub = node.advertise<geometry_msgs::PoseStamped>("ukf", 100);
	sub = node.subscribe("imu_data", 100, dataCallback);

	ros::spin();

	return 0;
}
