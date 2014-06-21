#ifndef StatusUpdater_h
#define StatusUpdater_h

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "blinky/RGB.h"
#include "blinky/UpdatePlannerLights.h"

class StatusUpdater {
	public:
		enum PossibleStates {ready, error, end, gate1, gate2, gate3, lane1, lane2, lane3, buoy1, buoy2, buoy3, hydro1, hydro2, hydro3,
												lost_gate1, lost_gate2, lost_gate3, lost_lane1};
		void updateStatus(PossibleStates newState);
		StatusUpdater(ros::Publisher frontEndPub, ros::ServiceClient btClient);

	private:
		ros::Publisher frontEndPublisher;
		ros::ServiceClient blinkyClient;
		blinky::RGB base;
		blinky::RGB off;
		blinky::RGB gate;
		blinky::RGB lane;
		blinky::RGB buoy;
		blinky::RGB hydro;
};

#endif