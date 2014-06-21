#include "Lost_Gate.h"

Lost_Gate::Lost_Gate(Planner* planner, StatusUpdater* mSU, int newPhase){
	myPlanner = planner;
	myStatusUpdater = mSU;
	phase = newPhase;
}

void Lost_Gate::execute() {
	switch(phase) {
		case 1:
			phase1();
			break;
		case 2:

			break;
		case 3:

			break;
	}
}

// assumes that if the robot still can't see the gate it's angle is off,
// so rotate until it sees it
void Lost_Gate::phase1() {
	ros::Rate loop_rate(50);
	loop_rate.sleep();

	myStatusUpdater->updateStatus(myStatusUpdater->lost_gate1);
	loop_rate.sleep();

	myPlanner->setVisionObj(1);
	loop_rate.sleep();
	
	myPlanner->setVelocity(0, 0, 2, 8.8, "/target/gate");
	loop_rate.sleep();
	tf::TransformListener listener;
	try {
		listener.waitForTransform("/target/gate", "/robot/rotation_center",
			ros::Time(0), ros::Duration(10));
	} catch (tf::TransformException ex) {
		myPlanner->switchToTask(myPlanner->Gate);
	}
}	

void Lost_Gate::phase0() {
	myPlanner->switchToTask(myPlanner->Gate);
}
