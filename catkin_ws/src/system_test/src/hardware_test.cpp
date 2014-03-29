#include "hardware_test.h"

int main(int argc, char **argv) {

	ros::init(argc, argv, "hardware_test");
	ros::NodeHandle nodeHandle;

	ROS_INFO("Initializing node %s", ros::this_node::getName().c_str());

	HardwareTest* hardwareTest = new HardwareTest();

	hardwareTest->runAllTests();

	delete hardwareTest;

	ros::shutdown();
}

HardwareTest::HardwareTest() {

}

HardwareTest::~HardwareTest() {

}

void HardwareTest::runAllTests() {
	printMainMenu();

	testVideoCameras();

	testDepthSensor();

	testIMU();

	testMainPVTemperatureSensor();

	testMainPVPressureSensor();

	testThrusters();
}

/**
 * Prints a header in the terminal using the ROS_INFO interface.
 *
 * @param headerToPrint The std::string object to print.
 */
void HardwareTest::printHeader(std::string headerToPrint) {
	int numberOfCharacters = headerToPrint.size();
	std::stringstream stars;
	for (int i = 0; i < (numberOfCharacters + 4); i++) {
		stars << "*";
	}
	ROS_INFO("%s", stars.str().c_str());
	ROS_INFO("%s", ("* " + headerToPrint + " *").c_str());
	ROS_INFO("%s", stars.str().c_str());
}

/**
 * Will ask the user to press a key in order to move on.
 */
void HardwareTest::pressAKey() {
	ROS_INFO("%s", "Press a key to continue...");
	std::cin.get();
}

void HardwareTest::printMainMenu() {
	printHeader("List of things that this program will test");
	ROS_INFO("%s", "1. Feeds from the cameras");
	ROS_INFO("%s", "2. Inputs from the depth sensor");
	ROS_INFO("%s", "3. Inputs from the IMU");
	ROS_INFO("%s", "4. Inputs from the temperature and pressure sensor");
	ROS_INFO("%s", "5. Left surge thruster");
	ROS_INFO("%s", "6. Right surge thruster");
	ROS_INFO("%s", "7. Front sway thruster");
	ROS_INFO("%s", "8. Back sway thruster");
	ROS_INFO("%s", "9. Front heave thruster");
	ROS_INFO("%s", "10. Back heave thruster");
	ROS_INFO("%s", "11. All thrusters running at the same time");
	ROS_INFO("%s", "12. LED arrays");
	pressAKey();
}

void HardwareTest::testVideoCameras() {
	printHeader("Testing the video feed from the camera");
	ROS_INFO("About to test the front left camera node, once you have visually confirmed that the camera was operational press CTRL+C once in order to terminate the background process.");
	pressAKey();
	system("roslaunch computer_vision camera_front_left.launch");
	pressAKey();
	ROS_INFO("About to test the front right camera node, once you have visually confirmed that the camera was operational press CTRL+C once in order to terminate the background process.");
	pressAKey();
	system("roslaunch computer_vision camera_front_left.launch");
	pressAKey();
	ROS_INFO("About to test the down camera node, once you have visually confirmed that the camera was operational press CTRL+C once in order to terminate the background process.");
	system("roslaunch computer_vision camera_down.launch");
	pressAKey();
}

void HardwareTest::testDepthSensor() {
	printHeader("Testing that we can receive data from the depth sensor");
	pressAKey();
}

void HardwareTest::testMainPVPressureSensor() {
	printHeader(
			"Testing that we can receive data from the pressure vessel's pressure sensor");
	pressAKey();
}

void HardwareTest::testMainPVTemperatureSensor() {
	printHeader(
			"Testing that we can receive data from the pressure vessel's temperature sensor");
	pressAKey();
}

void HardwareTest::testIMU() {
	printHeader("Testing that we can receive data from the IMU");
	pressAKey();
}

void HardwareTest::testThrusters() {
	printHeader("Testing the left surge thruster");
	pressAKey();
	ROS_INFO("%s", "Forward at 0%");
	pressAKey();
	ROS_INFO("%s", "Forward at 25%");
	pressAKey();
	ROS_INFO("%s", "Forward at 50%");
	pressAKey();
	ROS_INFO("%s", "Forward at 75%");
	pressAKey();
	ROS_INFO("%s", "Forward at 100%");
	pressAKey();
	ROS_INFO("%s", "Backward at 25%");
	pressAKey();
	ROS_INFO("%s", "Backward at 50%");
	pressAKey();
	ROS_INFO("%s", "Backward at 75%");
	pressAKey();
	ROS_INFO("%s", "Backward at 100%");
	pressAKey();
}
