#include "Lost_Lane.h"

Lost_Lane::Lost_Lane(Planner* planner, StatusUpdater* mSU, int newPhase){
	myPlanner = planner;
	myStatusUpdater = mSU;
	phase = newPhase;
}

void Lost_Lane::execute() {
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

void Lost_Lane::phase1() {
	ros::Rate loop_rate(50);
	loop_rate.sleep();

	myStatusUpdater->updateStatus(myStatusUpdater->lost_lane1);
	loop_rate.sleep();

	myPlanner->setVisionObj(2);
	loop_rate.sleep();
	
	myPlanner->setVelocity(-1, 0, 0, 8.8, "/robot/initial_pose");
	loop_rate.sleep();
	tf::TransformListener listener;
	try {
		listener.waitForTransform("/target/lane", "/robot/rotation_center",
			ros::Time(0), ros::Duration(10));
	} catch (tf::TransformException ex) {
		myPlanner->switchToTask(myPlanner->Lane);
	}
}