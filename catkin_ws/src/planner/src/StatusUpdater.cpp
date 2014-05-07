#include "StatusUpdater.h"

void StatusUpdater::updateFrontEnd(std::string currentStatus) {
	std_msgs::String msg;
	msg.data = currentStatus;
	frontEndPublisher.publish(msg);
}

void StatusUpdater::updateLED(blinky::RGB color) {
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

void StatusUpdater::updateStatus(PossibleStates newState) {
	std::string status;
	blinky::RGB color;
	switch(newState) {
		case startingGate:
			status = "Beginning Gate Task";
			//purple
			color.r = 255;
			color.g = 0;
			color.b = 255;
			break;
		case reachedGate:
			status = "Found the gate!";
			//blue
			color.r = 0;
			color.g = 0;
			color.b = 255;
			break;
		case completedGate:
			status = "Completed Gate Task";
			//green
			color.r = 0;
			color.g = 255;
			color.b = 0;
			break;
		case endRoutine:
			status = "Finished Routine";
			//black/off
			color.r = 0;
			color.g = 0;
			color.b = 0;
			break;
	}
	updateFrontEnd(status);
	updateLED(color);
}

StatusUpdater::StatusUpdater(ros::Publisher frontEndPub, ros::ServiceClient btClient) {
	frontEndPublisher = frontEndPub;
	blinkyClient = btClient;
}