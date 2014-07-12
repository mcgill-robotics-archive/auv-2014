#include "ros/ros.h"
#include "diagnostic_msgs/DiagnosticArray.h"
#include "string"
#include "stdlib.h"

/**
 * Camera Switch.
 * @author: Dwijesh Bhageerutty
 *
 * Usage: rosrun camera_switch camera_switch
 * This node subscribes to /diagnostics topic and monitors the message variable. 
 * When it is deemed that the camera connection is lost, that camera's node is killed.
 * The camera node is relaunched after a while and the above is repeated.
 * 
 * If the camera node dies 3 times, the spare camera's node is launched and this node shuts down itself.
 */
class CameraSwitch {
	public:
		/** 
		 *Construct a new CameraSwitch object and hook up this ROS node
		 */
		CameraSwitch(ros::NodeHandle& nh):
		node(nh)
		{
			diagnosticsSub = node.subscribe("diagnostics", 1, &CameraSwitch::diagnosticsCallback, this);
			frontRightDeathCount = 0;
			currentDiagnosticMessage = "";
		};
		
		/** 
		 * Process the incoming diagnostic message
		 */
		void diagnosticsCallback(const diagnostic_msgs::DiagnosticArray::ConstPtr& msg) {
			currentDiagnosticMessage = msg->status[0].message;
		};

		/**
		 * Loop
		 */
		void spin() {
			ros::Rate rate(10);

			while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C or node shuts down
			
				if ((currentDiagnosticMessage).compare("No events recorded.; No data since last update.") == 0) {

					// kill current node
					ROS_INFO( "Camera Front Right is no longer operational.");
					int result = system("rosnode kill /camera_front_right/camera1394_node");
					ROS_INFO("Camera's node succesfully killed.");
					
					frontRightDeathCount += 1;
				
					// sleep for 10 seconds
					ros::Duration(10).sleep();
				
					if (frontRightDeathCount <= 2) {
						// relaunch camera node
						ROS_INFO( "Re-launching camera node" );
						result = system("roslaunch computer_vision camera_front_right.launch &");

						ros::Duration(10).sleep();
					} else {
				
						// launch another node
						ROS_INFO( "Killed front_right 3 times. Launch spare camera's node" );
						result = system("roslaunch computer_vision camera_front_left.launch &");
						ros::shutdown();
					}
				}
			
				ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
				rate.sleep(); // Sleep for the rest of the cycle, to enforce the loop rate
			}
		};
	
	private:
		/** ros node handle */
		ros::NodeHandle& node;
		
		/** Subscriber for /diagnostics topic */
		ros::Subscriber diagnosticsSub;
		
		/** variable to keep track of the number of times the front right camera has been killed */
		int frontRightDeathCount;
		
		/** The current diagnostic message from the camera */
		std::string currentDiagnosticMessage;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "camera_switch");
    ros::NodeHandle n;
    CameraSwitch cameraSwitch(n);
	cameraSwitch.spin();
    return 0;
};
