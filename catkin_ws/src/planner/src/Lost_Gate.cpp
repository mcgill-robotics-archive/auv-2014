#include "Lost_Gate.h"

Lost_Gate::Lost_Gate(Planner* planner, StatusUpdater* mSU, int newPhase){
	myPlanner = planner;
	myStatusUpdater = mSU;
	phase = newPhase;
}

void Lost_Gate::execute() {
	ROS_INFO("yo, we lost");
	myPlanner->switchToTask(myPlanner->Gate);
}