#include "StatusUpdater.h"

void StatusUpdater::UpdateFrontEnd(std::string currentStatus) {
	std_msgs::String msg;
	msg.data = currentStatus;
	frontEndPublisher.publish(msg);
}

void StatusUpdater::UpdateLED(blinky::RGB color) {
	std::vector<blinky::RGB> colors;

	for(int i = 0; i < 30; i++) {
		colors.push_back(color);
	}
	
	blinky::UpdatePlannerLights srv;
	srv.request.colors = colors;
	if(!blinkyClient.call(srv)) {
		ROS_ERROR("failed to call service UpdatePlannerLights");
	}
}

void StatusUpdater::UpdateStatus(PossibleStates newState) {
	//use the enum to tell the above methods what to send out
}

StatusUpdater::StatusUpdater(ros::Publisher frontEndPub, ros::ServiceClient btClient) {
	frontEndPublisher = frontEndPub;
	blinkyClient = btClient;
}