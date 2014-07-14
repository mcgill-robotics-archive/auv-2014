#include "Task_Gate.h"

Task_Gate::Task_Gate(Planner* planner, StatusUpdater* mSU, int newPhase){
	myPlanner = planner;
	myStatusUpdater = mSU;
	phase = newPhase;
}

void Task_Gate::execute() {
	//setup stuff that should always run
	ROS_INFO("%s", "Executing the gate task.");

	if(phase <= 1) {
		phase1();
	}
	if(phase <= 2) {
		phase2();
	}
	if(phase <= 3) {
		phase3();
	}
}

//approach the gate area while looking for it
void Task_Gate::phase1() {
	frame = "/robot/initial_pose";

	ros::Rate loop_rate(50);
	loop_rate.sleep();

	myStatusUpdater->updateStatus(myStatusUpdater->gate1);
	ROS_INFO("Gate phase 1");
	loop_rate.sleep();

	myPlanner->setVisionObj(1);
	loop_rate.sleep();
	
	while (!myPlanner->getSeeObject()) {
		ROS_INFO_THROTTLE(1, "Does not see gate, sending velocity with close loop yaw and depth.");
		myPlanner->setVelocityWithCloseLoopYawAndDepth(0, -4, 8.8, frame);
		loop_rate.sleep();
		ros::spinOnce();
	}

	/*
	tf::TransformListener listener;
	try {
		listener.waitForTransform(frame, "/robot/rotation_center",
			ros::Time(0), ros::Duration(90));
	} catch (tf::TransformException ex) {
		ROS_INFO("Could not find transform from %s to /robot/rotation_center, keep going straight", frame.c_str());
		myPlanner->setVelocityWithCloseLoopYawAndDepth(0, 1, 8.8, frame); //Keeps going straight if cannot find transform
	}
	*/
	ROS_INFO("we found it");
}

//after finding the gate, approach it even more using CV data
void Task_Gate::phase2() {	
	frame = "/target/gate";

	//TODO: update robot's reference frame with CV data

	ros::Rate loop_rate(50);
	loop_rate.sleep();
	
	myStatusUpdater->updateStatus(myStatusUpdater->gate2);
	ROS_INFO("Gate phase 2");
	loop_rate.sleep();
	
	double myPoints[5] = {2.5, 0.0, 0.0, 0.0, 8.8};
	std::vector<double> desired(myPoints, myPoints + sizeof(myPoints) / sizeof(myPoints[0]));
	
	while (!myPlanner->areWeThereYet(frame, desired)) {
		ROS_DEBUG("Task_Gate::setPoints published");		
		myPlanner->setPosition(desired, frame);
		loop_rate.sleep();
	}
}

//move through the gate
void Task_Gate::phase3() {
	//TODO: use a frame updated in phase 2
	frame = "/robot/initial_pose";

	ros::Rate loop_rate(50);
	loop_rate.sleep();

	myStatusUpdater->updateStatus(myStatusUpdater->gate3);
	ROS_INFO("Gate phase 3");
	loop_rate.sleep();

	ROS_INFO("Task_Gate::reached the front of the gate");
	//TODO: this phase doesn't really do anything anymore.
	//Controls requires continuous sending of velocity now.
	myPlanner->setVelocityWithCloseLoopYawAndDepth(0, 1, 8.8, frame);

	ROS_INFO("%s", "Task_Gate::The gate task has been completed.");
	loop_rate.sleep();

	//next task
	myPlanner->switchToTask(myPlanner->Lane);
}