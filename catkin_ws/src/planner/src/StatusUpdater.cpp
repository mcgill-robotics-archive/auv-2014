#include "StatusUpdater.h"

void StatusUpdater::updateFrontEnd(std::string currentStatus) {
	std_msgs::String msg;
	msg.data = currentStatus;
	frontEndPublisher.publish(msg);
}

void StatusUpdater::updateLED(BlinkyStates newState) {
	blinky::RGB color1;
	blinky::RGB color2;
	switch(newState) {
		case openLoop:
			//purple
			color1.r = 161;
			color1.g = 15;
			color1.b = 127;
			color2 = color1;
			break;
		case closedLoop:
			//green
			color1.r = 0;
			color1.g = 255;
			color1.b = 0;
			color2 = color1;
			break;
		case taskAction:
			//orange
			color1.r = 255;
			color1.g = 102;
			color1.b = 0;
			color2 = color1;
			break;
		case taskComplete:
			//yellow
			color1.r = 255;
			color1.g = 255;
			color1.b = 0;
			color2 = color1;
			break;
		case cleanFinish:
			//white
			color1.r = 255;
			color1.g = 255;
			color1.b = 255;
			color2 = color1;
			break;
		case error:
			//black/off
			color1.r = 0;
			color1.g = 0;
			color1.b = 0;
			color2 = color1;
			break;
		case readyToGo:
			//white & blue
			color1.r = 255;
			color1.g = 255;
			color1.b = 255;
			color2.r = 0;
			color2.g = 0;
			color2.b = 255;
			break;
	}

	std::vector<blinky::RGB> colors;

	for(int i = 0; i < 30; i++) {
		colors.push_back(color1);
		colors.push_back(color2);
	}
	
	blinky::UpdatePlannerLights srv;
	srv.request.colors = colors;
	if(!blinkyClient.call(srv)) {
		ROS_ERROR("failed to call service UpdatePlannerLights");
	}
}

void StatusUpdater::updateStatus(PossibleStates newState) {
	std::string status;
	BlinkyStates color;
	switch(newState) {
		case readyToStart:
			status = "ready to start the run";
			color = readyToGo;
		case startingGate:
			status = "Beginning Gate Task";
			color = taskComplete;
			break;
		case reachedGate:
			status = "Found the gate!";
			color = closedLoop;
			break;
		case completedGate:
			status = "Completed Gate Task";
			color = taskComplete;
			break;
		case endRoutine:
			status = "Finished Routine";
			color = cleanFinish;
			break;
	}
	updateFrontEnd(status);
	updateLED(color);
}

StatusUpdater::StatusUpdater(ros::Publisher frontEndPub, ros::ServiceClient btClient) {
	frontEndPublisher = frontEndPub;
	blinkyClient = btClient;
}