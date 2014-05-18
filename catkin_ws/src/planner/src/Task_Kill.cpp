#include "Task_Kill.h"

Task_Kill::Task_Kill(Planner* planner, StatusUpdater* mSU, int newPhase){
	myPlanner = planner;
	myStatusUpdater = mSU;
	phase = newPhase;
}

void Task_Kill::execute() {
	//currently does absolutely nothing
	ros::Rate loop_rate(50);
	loop_rate.sleep();

	myPlanner->setVelocity(0,0,0,0,"/robot/rotation_center");
	loop_rate.sleep();
	
	myStatusUpdater->updateStatus(myStatusUpdater->endRoutine);
	loop_rate.sleep();
}