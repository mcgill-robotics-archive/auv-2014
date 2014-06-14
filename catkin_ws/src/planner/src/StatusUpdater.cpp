#include "StatusUpdater.h"

void StatusUpdater::updateStatus(PossibleStates newState) {
	std_msgs::String msg;
	blinky::RGB color1;
	blinky::RGB color2;
	blinky::RGB color3;
	switch(newState) {
		case ready:
			msg.data = "ready to start the run";
			//all base
			color1 = base;
			color2 = base;
			color3 = base;
			break;
		case gate1:
			msg.data = "Looking for the gate";
			//all green
			color1 = gate;
			color2 = gate;
			color3 = gate;
			break;
		case gate2:
			msg.data = "Found the gate!";
			//2 green, 1 base
			color1 = gate;
			color2 = gate;
			color3 = base;
			break;
		case gate3:
			msg.data = "going through the gate";
			//1 green, 2 base
			color1 = gate;
			color2 = base;
			color3 = base;
			break;
		case lane1:
			msg.data = "looking for the lane";
			//all purple
			color1 = lane;
			color2 = lane;
			color3 = lane;
			break;
		case lane2:
			msg.data = "Found the lane!";
			//2 purple, 1 base
			color1 = lane;
			color2 = lane;
			color3 = base;
			break;
		case lane3:
			msg.data = "leaving the lane";
			//1 purple, 2 base
			color1 = lane;
			color2 = base;
			color3 = base;
			break;
		case buoy1:
			msg.data = "Beginning Buoy Task";
			//all yellow
			color1 = buoy;
			color2 = buoy;
			color3 = buoy;
			break;
		case buoy2:
			msg.data = "Found the buoy!";
			//2 yellow, 1 base
			color1 = buoy;
			color2 = buoy;
			color3 = base;
			break;
		case buoy3:
			msg.data = "Completed Buoy Task";
			color1 = buoy;
			color2 = base;
			color3 = base;
			break;
		case hydro1:
			msg.data = "Beginning hydrophones Task";
			color1 = hydro;
			color2 = hydro;
			color3 = hydro;
			break;
		case hydro2:
			msg.data = "Found the pinger!";
			color1 = hydro;
			color2 = hydro;
			color3 = base;
			break;
		case hydro3:
			msg.data = "Completed hydrophones Task";
			color1 = hydro;
			color2 = base;
			color3 = base;
			break;
		case error:
			msg.data = "error encountered";
			color1 = off;
			color2 = off;
			color3 = off;
			break;	
		case end:
			msg.data = "Finished Routine";
			color1 = base;
			color2 = base;
			color3 = base;
			break;
	}
	frontEndPublisher.publish(msg);

	std::vector<blinky::RGB> colors;
/*
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
*/
colors.push_back(color1);
colors.push_back(color2);
colors.push_back(color3);

	blinky::UpdatePlannerLights srv;
	srv.request.colors = colors;
	if(!blinkyClient.call(srv)) {
		ROS_ERROR("failed to call service UpdatePlannerLights");
	}
}

StatusUpdater::StatusUpdater(ros::Publisher frontEndPub, ros::ServiceClient btClient) {
	frontEndPublisher = frontEndPub;
	blinkyClient = btClient;
	//cyan
	base.r = 0;
	base.g = 255;
	base.b = 255;
	//black
	off.r = 0;
	off.g = 0;
	off.b = 0;
	//green
	gate.r = 0;
	gate.g = 255;
	gate.b = 0;
	//yellow
	lane.r = 255;
	lane.g = 200;
	lane.b = 0;
	//orange?
	buoy.r = 0;
	buoy.g = 0;
	buoy.b = 0;
	//purple
	hydro.r = 255;
	hydro.g = 0;
	hydro.b = 125;
}