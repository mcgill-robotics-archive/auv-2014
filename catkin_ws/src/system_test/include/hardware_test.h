#include <ros/ros.h>
#include <ros/callback_queue.h>
#include "controls/motorCommands.h"

class HardwareTest {
private:
	ros::Publisher motorCommandsPublisher;

private:
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
	void testLEDs();

public:
	HardwareTest(ros::NodeHandle& nodeHandle);
	~HardwareTest();
	void runAllTests();
};
