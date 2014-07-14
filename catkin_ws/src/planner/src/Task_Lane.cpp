#include "Task_Lane.h"

Task_Lane::Task_Lane(Planner* planner, StatusUpdater* mSU, int newPhase){
	myPlanner = planner;
	myStatusUpdater = mSU;
	phase = newPhase;
}

void Task_Lane::execute() {
	std::string frame = "/target/lane";

	ros::Rate loop_rate(50);
	loop_rate.sleep();
	//look for the lane marker
	myPlanner->setVisionObj(2);
	myStatusUpdater->updateStatus(myStatusUpdater->lane1);

	loop_rate.sleep();
    ROS_INFO("looking for the lane");

    while (!myPlanner->getSeeObject()) {
		myPlanner->setVelocityWithCloseLoopYawPitchDepth(-4, 0, 0, 8.8, frame);
        loop_rate.sleep();
    }

	tf::TransformListener listener;
	listener.waitForTransform(frame, "/robot/rotation_center",
			ros::Time(0), ros::Duration(300));
//stops moving
//	double myPoints0[5] = {0.0, 0.0, 0.0, 0.0, 8.8};
//	std::vector<double> desired0(myPoints0, myPoints0 + sizeof(myPoints0) / sizeof(myPoints0[0]));
//	myPlanner->setPosition(desired0, "/robot/rotation_center");
//	myPlanner->setVelocity(0, 0, 0, 8.8, frame);

    ROS_INFO("found the lane");
	myStatusUpdater->updateStatus(myStatusUpdater->lane2);
	loop_rate.sleep();
	//position above the lane marker
	double myPoints[5] = {0.0, 0.0, 0.0, 0.0, 8.8};
	std::vector<double> desired(myPoints, myPoints + sizeof(myPoints) / sizeof(myPoints[0]));

	while (!myPlanner->areWeThereYet(frame, desired)) {
		ROS_INFO("NOT THERE YET!!!!");
		ROS_DEBUG("Task_Lane::setPoints published");		
		myPlanner->setPosition(desired, frame);
		loop_rate.sleep();
	}
	myStatusUpdater->updateStatus(myStatusUpdater->lane3);
	loop_rate.sleep();
	//end the routine
	myPlanner->switchToTask(myPlanner->Kill);
}