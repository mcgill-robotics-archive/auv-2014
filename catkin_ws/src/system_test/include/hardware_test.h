#include <ros/ros.h>
#include <ros/callback_queue.h>

class HardwareTest {
private:
	void printHeader(std::string headerToPrint);
	void pressAKey();
	void printMainMenu();
	void testVideoCameras();
	void testDepthSensor();
	void testMainPVPressureSensor();
	void testMainPVTemperatureSensor();
	void testIMU();
	void testThrusters();

public:
	HardwareTest();
	~HardwareTest();
	void runAllTests();
};
