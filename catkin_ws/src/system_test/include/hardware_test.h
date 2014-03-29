#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "controls/motorCommands.h"
#include "std_msgs/Float32.h"

// Static variables.
const int MAX_NUMBER_OF_DEPTH_SENSOR_READINGS = 50;

class HardwareTest {
private: // Private variables.
	ros::NodeHandle nodeHandle;
	ros::Publisher motorCommandsPublisher;
	ros::Subscriber depthSensorDataSubscriber;
	std::string motorCommandTopicName;
	std::string depthSensorDataTopicName;

	int numberOfReadingsFromDepthSensor;

private: // Private methods.
	void printHeader(std::string headerToPrint);
	void pressAKey();
	void printMainMenu();
	void testVideoCameras();
	void testDepthSensor();
	void testMainPVPressureSensor();
	void testMainPVTemperatureSensor();
	void testIMU();
	void testAllThrusters();
	void testLeftSurgeThrusters();
	void testRightSurgeThrusters();
	void testBackSwayThrusters();
	void testFrontSwayThrusters();
	void testFrontHeaveThrusters();
	void testBackHeaveThrusters();
	void testLEDs();
	void depthSensorCallback(std_msgs::Float32 readingFromDepthSensor);

public: // Public methods.
	HardwareTest(ros::NodeHandle& nodeHandle);
	~HardwareTest();
	void runAllTests();
};
