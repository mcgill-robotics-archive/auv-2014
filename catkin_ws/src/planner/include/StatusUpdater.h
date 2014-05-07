#ifndef StatusUpdater_h
#define StatusUpdater_h

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "blinky/RGB.h"
#include "blinky/UpdatePlannerLights.h"

class StatusUpdater {
	public:
		enum PossibleStates {startingGate, reachedGate, completedGate, endRoutine, etc};
		void updateFrontEnd(std::string currentStatus);
		void updateLED(blinky::RGB color);
		void updateStatus(PossibleStates newState);
		StatusUpdater(ros::Publisher frontEndPub, ros::ServiceClient btClient);

	private:
		ros::Publisher frontEndPublisher;
		ros::ServiceClient blinkyClient;
};

#endif