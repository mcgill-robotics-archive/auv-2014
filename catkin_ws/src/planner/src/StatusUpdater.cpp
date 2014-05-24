#include "StatusUpdater.h"

void StatusUpdater::updateStatus(PossibleStates newState) {
	std_msgs::String msg;
	//the base color
	blinky::RGB baseColor;
	baseColor.r = 0;
	baseColor.g = 0;
	baseColor.b = 0;
	//off
	blinky::RGB off;
	off.r = 0;
	off.g = 0;
	off.b = 0;
	
	blinky::RGB color1;
	blinky::RGB color2;
	blinky::RGB color3;
	switch(newState) {
		case ready:
			msg.data = "ready to start the run";
			//all base
			color1 = baseColor;
			color2 = baseColor;
			color3 = baseColor;
			break;
		case gate1:
			msg.data = "Looking for the gate";
			//all green
			color1.r = 0;
			color1.g = 255;
			color1.b = 0;
			color2 = color1;
			color3 = color1;
			break;
		case gate2:
			msg.data = "Found the gate!";
			//2 green, 1 base
			color1.r = 0;
			color1.g = 255;
			color1.b = 0;
			color2 = color1;
			color3 = baseColor;
			break;
		case gate3:
			msg.data = "going through the gate";
			//1 green, 2 base
			color1.r = 0;
			color1.g = 255;
			color1.b = 0;
			color2 = baseColor;
			color3 = baseColor;
			break;
		case lane1:
			msg.data = "looking for the lane";
			//all purple
			color1.r = 161;
			color1.g = 15;
			color1.b = 127;
			color2 = color1;
			color3 = color1;
			break;
		case lane2:
			msg.data = "Found the lane!";
			//2 purple, 1 base
			color1.r = 161;
			color1.g = 15;
			color1.b = 127;
			color2 = color1;
			color3 = baseColor;
			break;
		case lane3:
			msg.data = "leaving the lane";
			//1 purple, 2 base
			color1.r = 161;
			color1.g = 15;
			color1.b = 127;
			color2 = baseColor;
			color3 = baseColor;
			break;
		case buoy1:
			msg.data = "Beginning Buoy Task";
			//all yellow
			color1.r = 255;
			color1.g = 255;
			color1.b = 0;
			color2 = color1;
			color3 = color1;
			break;
		case buoy2:
			msg.data = "Found the buoy!";
			//2 yellow, 1 base
			color1.r = 255;
			color1.g = 255;
			color1.b = 0;
			color2 = color1;
			color3 = baseColor;
			break;
		case buoy3:
			msg.data = "Completed Buoy Task";
			//1 yellow, 2 base
			color1.r = 255;
			color1.g = 255;
			color1.b = 0;
			color2 = baseColor;
			color3 = baseColor;
			break;
		case error:
			msg.data = "error encountered";
			//all off
			color1 = off;
			color2 = off;
			color3 = off;
			break;	
		case end:
			msg.data = "Finished Routine";
			//all white
			color1.r = 255;
			color1.g = 255;
			color1.b = 255;
			color2 = color1;
			color3 = color1;
			break;
	}
	frontEndPublisher.publish(msg);

	std::vector<blinky::RGB> colors;

	for(int i = 0; i < 5; i++) {
		colors.push_back(color1);
	}
	colors.push_back(off);
	for(int i = 0; i < 4; i++) {
		colors.push_back(color2);
	}
	colors.push_back(off);
	for(int i = 0; i < 4; i++) {
		colors.push_back(color3);
	}
	
	blinky::UpdatePlannerLights srv;
	srv.request.colors = colors;
	if(!blinkyClient.call(srv)) {
		ROS_ERROR("failed to call service UpdatePlannerLights");
	}
}

StatusUpdater::StatusUpdater(ros::Publisher frontEndPub, ros::ServiceClient btClient) {
	frontEndPublisher = frontEndPub;
	blinkyClient = btClient;
}