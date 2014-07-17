#include "Task_Lane.h"

Task_Lane::Task_Lane(Planner* planner, StatusUpdater* mSU, int newPhase){
	myPlanner = planner;
	myStatusUpdater = mSU;
	phase = newPhase;
}

void Task_Lane::execute() {
	std::string startFrame = "/sensors/IMU_global_reference";
	std::string frame = "/target/lane";

	ros::Rate loop_rate(50);
	loop_rate.sleep();
	//look for the lane marker
	myPlanner->setVisionObj(2);
	myStatusUpdater->updateStatus(myStatusUpdater->lane1);

	loop_rate.sleep();
    ROS_INFO("looking for the lane");

    while (!myPlanner->getSeeObject()) {
		myPlanner->setVelocityWithCloseLoopYawPitchDepth(myPlanner->getSurgeSpeed(), 0, 0, myPlanner->getCloseLoopDepth(), startFrame);
        loop_rate.sleep();
    }

	ROS_INFO("FOUND THE LANE. Going to wait for transform");

	tf::TransformListener listener;
	listener.waitForTransform(frame, "/robot/rotation_center",
			ros::Time(0), ros::Duration(0.4));
 
    ROS_INFO("Got the transform");
	myStatusUpdater->updateStatus(myStatusUpdater->lane2);
	loop_rate.sleep();
	//position above the lane marker
	double myPoints[5] = {0.0, 0.0, 0.0, 0.0, myPlanner->getCloseLoopDepth()};
	std::vector<double> desired(myPoints, myPoints + sizeof(myPoints) / sizeof(myPoints[0]));

	while (!myPlanner->areWeThereYet(frame, desired)) {
		ROS_DEBUG("Task_Lane::setPoints published");		
		myPlanner->setPosition(desired, frame);
		loop_rate.sleep();
	}
	myStatusUpdater->updateStatus(myStatusUpdater->lane3);
	loop_rate.sleep();

	//end the routine
	//myPlanner->switchToTask(myPlanner->Kill);
}