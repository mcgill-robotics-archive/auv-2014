#include "hardware_test.h"

int main(int argc, char **argv) {

	ros::init(argc, argv, "hardware_test");
	ros::NodeHandle nodeHandle;

	ROS_INFO("Initializing node %s", ros::this_node::getName().c_str());

	HardwareTest* hardwareTest = new HardwareTest(nodeHandle);

	hardwareTest->runAllTests();

	delete hardwareTest;

	ros::shutdown();
}

HardwareTest::HardwareTest(ros::NodeHandle& nodeHandle) {
	motorCommandsPublisher = nodeHandle.advertise<controls::motorCommands>("/arduino/motor", 10);
}

HardwareTest::~HardwareTest() {
}

void HardwareTest::runAllTests() {
	printMainMenu();

	testAllThrusters();

	testVideoCameras();

	testDepthSensor();

	testIMU();

	testMainPVTemperatureSensor();

	testMainPVPressureSensor();
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
	system("clear");
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
	printHeader("Testing the cameras");
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
	printHeader("Testing the depth sensor");
	pressAKey();
}

void HardwareTest::testMainPVPressureSensor() {
	printHeader("Testing the main PV pressure sensor");
	pressAKey();
}

void HardwareTest::testMainPVTemperatureSensor() {
	printHeader("Testing the main PV temperature sensor");
	pressAKey();
}

void HardwareTest::testIMU() {
	printHeader("Testing the IMU");
	pressAKey();
}

void HardwareTest::testAllThrusters() {
	testLeftSurgeThrusters();
	testRightSurgeThrusters();
	testFrontSwayThrusters();
	testBackSwayThrusters();
	testFrontHeaveThrusters();
	testBackHeaveThrusters();
}

void HardwareTest::testLeftSurgeThrusters() {
	printHeader("Testing the left surge thruster");

	controls::motorCommands* motorCommands = new controls::motorCommands();

	pressAKey();
	ROS_INFO("%s", "Speed at 0%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Forward at 25%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = (int32_t)((0.25)*(double)500);
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Forward at 50%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = (int32_t)((0.50)*(double)500);
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Forward at 75%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = (int32_t)((0.75)*(double)500);
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Forward at 100%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = (int32_t)((1.00)*(double)500);
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Speed at 0%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Backward at 25%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = (int32_t)((0.25)*(double)-500);
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Backward at 50%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = (int32_t)((0.50)*(double)-500);
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Backward at 75%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = (int32_t)((0.75)*(double)-500);
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Backward at 100%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = (int32_t)((1.00)*(double)-500);
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Speed at 0%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Testing positive out of bound value");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = (int32_t)((1.25)*(double)500);
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Testing negative out of bound value");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = (int32_t)((1.25)*(double)-500);
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();

	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);

	delete motorCommands;
}

void HardwareTest::testRightSurgeThrusters() {
	printHeader("Testing the right surge thruster");

	controls::motorCommands* motorCommands = new controls::motorCommands();

	pressAKey();
	ROS_INFO("%s", "Speed at 0%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Forward at 25%");
	motorCommands->cmd_x1 = (int32_t)((0.25)*(double)500);
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Forward at 50%");
	motorCommands->cmd_x1 = (int32_t)((0.50)*(double)500);
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Forward at 75%");
	motorCommands->cmd_x1 = (int32_t)((0.75)*(double)500);
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Forward at 100%");
	motorCommands->cmd_x1 = (int32_t)((1.00)*(double)500);
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Speed at 0%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Backward at 25%");
	motorCommands->cmd_x1 = (int32_t)((0.25)*(double)-500);
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Backward at 50%");
	motorCommands->cmd_x1 = (int32_t)((0.50)*(double)-500);
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Backward at 75%");
	motorCommands->cmd_x1 = (int32_t)((0.75)*(double)-500);
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Backward at 100%");
	motorCommands->cmd_x1 = (int32_t)((1.00)*(double)-500);
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Speed at 0%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Testing positive out of bound value");
	motorCommands->cmd_x1 = (int32_t)((1.25)*(double)500);
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Testing negative out of bound value");
	motorCommands->cmd_x1 = (int32_t)((1.25)*(double)-500);
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();

	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);

	delete motorCommands;
}

void HardwareTest::testFrontSwayThrusters() {
	printHeader("Testing the front sway thruster");

	controls::motorCommands* motorCommands = new controls::motorCommands();

	pressAKey();
	ROS_INFO("%s", "Speed at 0%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Forward at 25%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = (int32_t)((0.25)*(double)500);
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Forward at 50%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = (int32_t)((0.50)*(double)500);
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Forward at 75%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = (int32_t)((0.75)*(double)500);
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Forward at 100%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = (int32_t)((1.00)*(double)500);
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Speed at 0%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Backward at 25%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = (int32_t)((0.25)*(double)-500);
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Backward at 50%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = (int32_t)((0.50)*(double)-500);
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Backward at 75%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = (int32_t)((0.75)*(double)-500);
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Backward at 100%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = (int32_t)((1.00)*(double)-500);
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Speed at 0%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Testing positive out of bound value");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = (int32_t)((1.25)*(double)500);
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Testing negative out of bound value");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = (int32_t)((1.25)*(double)-500);
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();

	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);

	delete motorCommands;
}

void HardwareTest::testBackSwayThrusters() {
	printHeader("Testing the back sway thruster");

	controls::motorCommands* motorCommands = new controls::motorCommands();

	pressAKey();
	ROS_INFO("%s", "Speed at 0%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Forward at 25%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = (int32_t)((0.25)*(double)500);
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Forward at 50%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = (int32_t)((0.50)*(double)500);
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Forward at 75%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = (int32_t)((0.75)*(double)500);
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Forward at 100%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = (int32_t)((1.00)*(double)500);
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Speed at 0%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Backward at 25%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = (int32_t)((0.25)*(double)-500);
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Backward at 50%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = (int32_t)((0.50)*(double)-500);
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Backward at 75%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = (int32_t)((0.75)*(double)-500);
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Backward at 100%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = (int32_t)((1.00)*(double)-500);
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Speed at 0%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Testing positive out of bound value");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = (int32_t)((1.25)*(double)500);
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Testing negative out of bound value");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = (int32_t)((1.25)*(double)-500);
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();

	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);

	delete motorCommands;
}

void HardwareTest::testFrontHeaveThrusters() {
	printHeader("Testing the front sway thruster");

	controls::motorCommands* motorCommands = new controls::motorCommands();

	pressAKey();
	ROS_INFO("%s", "Speed at 0%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Forward at 25%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = (int32_t)((0.25)*(double)500);
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Forward at 50%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = (int32_t)((0.50)*(double)500);
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Forward at 75%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = (int32_t)((0.75)*(double)500);
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Forward at 100%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = (int32_t)((1.00)*(double)500);
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Speed at 0%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Backward at 25%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = (int32_t)((0.25)*(double)-500);
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Backward at 50%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = (int32_t)((0.50)*(double)-500);
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Backward at 75%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = (int32_t)((0.75)*(double)-500);
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Backward at 100%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = (int32_t)((1.00)*(double)-500);
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Speed at 0%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Testing positive out of bound value");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = (int32_t)((1.25)*(double)500);
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Testing negative out of bound value");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = (int32_t)((1.25)*(double)-500);
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();

	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);

	delete motorCommands;
}

void HardwareTest::testBackHeaveThrusters() {
	printHeader("Testing the back sway thruster");

	controls::motorCommands* motorCommands = new controls::motorCommands();

	pressAKey();
	ROS_INFO("%s", "Speed at 0%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Forward at 25%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = (int32_t)((0.25)*(double)500);
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Forward at 50%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = (int32_t)((0.50)*(double)500);
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Forward at 75%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = (int32_t)((0.75)*(double)500);
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Forward at 100%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = (int32_t)((1.00)*(double)500);
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Speed at 0%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Backward at 25%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = (int32_t)((0.25)*(double)-500);
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Backward at 50%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = (int32_t)((0.50)*(double)-500);
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Backward at 75%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = (int32_t)((0.75)*(double)-500);
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Backward at 100%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = (int32_t)((1.00)*(double)-500);
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Speed at 0%");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Testing positive out of bound value");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = (int32_t)((1.25)*(double)500);
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();
	ROS_INFO("%s", "Testing negative out of bound value");
	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = (int32_t)((1.25)*(double)-500);
	motorCommandsPublisher.publish(*motorCommands);
	pressAKey();

	motorCommands->cmd_x1 = 0;
	motorCommands->cmd_x2 = 0;
	motorCommands->cmd_y1 = 0;
	motorCommands->cmd_y2 = 0;
	motorCommands->cmd_z1 = 0;
	motorCommands->cmd_z2 = 0;
	motorCommandsPublisher.publish(*motorCommands);

	delete motorCommands;
}

void HardwareTest::testLEDs() {
	printHeader("Testing the LEDs");
	pressAKey();
}
