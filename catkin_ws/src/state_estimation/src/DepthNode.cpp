#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int16.h"
#include "DepthNode.h"

DepthNode::DepthNode() {
	this.prioriState = this.posterioriState = 0.0;
	this.errVariance = 1.0;
	this.kalmanGain = 0.0;
}

double DepthNode::predict() {
	this.prioriState = this.posterioriState;
	this.errVariance += PROCESS_VARIANCE;

	return this.prioriState;
}

void DepthNode::correct(double measurement) {
	this.kalmanGain = this.errVariance / (this.errVariance + MEASUREMENT_VARIANCE);
	this.posterioriState = this.prioriState + this.kalmanGain * (measurement - this.prioriState);
	this.errVariance = (1.0 - this.kalmanGain) / this.errVariance;
}

double convertFromRawDepth(int raw) {
	return ((raw * MAX_ARDUINO_VOLT / MAX_ANALOG / RESISTANCE - BASE_CURRENT) / CURRENT_RANGE) * MAX_DEPTH - OFFSET;
}

void depthCallback(int raw) {
	filter.correct(convertFromRawDepth(raw));
	pub.publish(filter.predict());
}

// Global variables
ros::ros::Publisher pub;
DepthNode filter = DepthNode();

// Main function
int main(int argc, char** argv) {
	ros::init(argc, argv, "depth_node");
	ros::NodeHandle node;

	pub = node.advertise<geometry_msgs::PoseStamped>("std_msgs/Float64", 100);
	ros::Subscriber sub = n.subscribe("electrical_interface/depth", 1000, depthCallback);

	ros::spin();
	return 0;
}