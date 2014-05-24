#ifndef StatusUpdater_h
#define StatusUpdater_h

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "blinky/RGB.h"
#include "blinky/UpdatePlannerLights.h"

class StatusUpdater {
	public:
		enum PossibleStates {ready, gate1, gate2, gate3, lane1, lane2, lane3, buoy1, buoy2, buoy3, error, end};
		void updateStatus(PossibleStates newState);
		StatusUpdater(ros::Publisher frontEndPub, ros::ServiceClient btClient);

	private:
		ros::Publisher frontEndPublisher;
		ros::ServiceClient blinkyClient;
};

#endif