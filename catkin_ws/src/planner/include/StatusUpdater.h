#ifndef StatusUpdater_h
#define StatusUpdater_h

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "blinky/RGB.h"
#include "blinky/UpdatePlannerLights.h"

class StatusUpdater {
	public:
		enum PossibleStates {readyToStart, startingGate, reachedGate, completedGate, endRoutine, etc};
		enum BlinkyStates {openLoop, closedLoop, taskAction, taskComplete, cleanFinish, error, readyToGo};
		void updateFrontEnd(std::string currentStatus);
		void updateLED(BlinkyStates newState);
		void updateStatus(PossibleStates newState);
		StatusUpdater(ros::Publisher frontEndPub, ros::ServiceClient btClient);

	private:
		ros::Publisher frontEndPublisher;
		ros::ServiceClient blinkyClient;
};

#endif