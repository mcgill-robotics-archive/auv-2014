#ifndef StatusUpdater_h
#define StatusUpdater_h

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "blinky/RGB.h"
#include "blinky/UpdatePlannerLights.h"

class StatusUpdater {
	public:
		enum PossibleStates {startingGate, doingGate, completingGate, etc};
		void UpdateFrontEnd(std::string currentStatus);
		void UpdateLED(blinky::RGB color);
		void UpdateStatus(PossibleStates newState);
		StatusUpdater(ros::Publisher frontEndPub, ros::ServiceClient btClient);

	private:
		ros::Publisher frontEndPublisher;
		ros::ServiceClient blinkyClient;
};

#endif