#include "ros/ros.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "string"
#include "stdlib.h"


/**
 * Class to test switching from a camera to another.
 * @author: Dwijesh Bhageerutty
 *
 * Usage: rosrun test_camera_switch test_camera_switch
 * This node subscribes to /diagnostics and when a certain diagnostics status message is encountered
 * kills the camera_front_right node. Then it kills itself (that's just for now).
 *
 * Notes: right now using system() to call rosnode kill - need to figure out a better way of doing this.
 */
class TestCameraSwitch {
	public:
		// Construct a new TestCameraSwitch object and hook up this ROS node
		TestCameraSwitch(ros::NodeHandle& nh):
		node(nh)
		{
			diagnosticsSub = node.subscribe("diagnostics", 1, &TestCameraSwitch::diagnosticsCallback, this);
		};
		
		// Process the incoming laser scan message
		void diagnosticsCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg) {
			if ((msg->status[0].message).compare("No events recorded.; No data since last update.") == 0) {
				std::cout << "Camera Front Right no longer operational." << std::endl;
				int result = system("rosnode kill /camera_front_right/camera1394_node");
				std::cout << "system call result: " << result << std::endl;
				ros::shutdown();
			}
		};

		void spin() {
			ros::Rate rate(10); // Specify the FSM loop rate in Hz
			while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
				ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
				rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
			}
		};
	
	private:
		/** ros node handle */
		ros::NodeHandle& node;
		
		/** Subscriber for /diagnostics topic */
		ros::Subscriber diagnosticsSub;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_switch");
    ros::NodeHandle n;
    TestCameraSwitch cameraSwitch(n);
	cameraSwitch.spin();
    return 0;
};
