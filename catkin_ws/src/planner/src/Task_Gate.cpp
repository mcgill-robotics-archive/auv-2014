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

	/*
	Disable gate CV for competition

	if(phase <= 2) {
		phase2();
	}
	if(phase <= 3) {
		phase3();
	}
	*/
}

//approach the gate area while looking for it
void Task_Gate::phase1() {
	frame = "/sensors/IMU_global_reference";

	ros::Rate loop_rate(50);
	loop_rate.sleep();

	myStatusUpdater->updateStatus(myStatusUpdater->gate1);
	ROS_INFO("Gate phase 1");
	loop_rate.sleep();

	myPlanner->setVisionObj(1);
	loop_rate.sleep();

	double yawTimeout = myPlanner -> getYawTimeout();
	ROS_INFO("Sending Yaw = 0 for %f seconds", yawTimeout);
	ros::Time start_time = ros::Time::now();
	ros::Duration timeout(yawTimeout);
	while(ros::Time::now() - start_time < timeout) {
	        ROS_INFO_THROTTLE(1, "Sending yaw = 0");
	        myPlanner->setYaw(0, frame);
	        loop_rate.sleep();
	        ros::spinOnce();
	}

	if (myPlanner->isOpenLoopDepth()) {
		//Wait some time before sending surge
		double openLoopDepthTimeout = myPlanner -> getOpenLoopDepthTimeout();
		ROS_INFO("OPEN LOOP DEPTH and close loop yaw and pitch for %f seconds", openLoopDepthTimeout);
		start_time = ros::Time::now();
		timeout =ros::Duration(openLoopDepthTimeout);
		while(ros::Time::now() - start_time < timeout) {
	        ROS_INFO_THROTTLE(1, "Sending depth, yaw, pitch first, before surge. %f seconds left", (timeout - (ros::Time::now() - start_time)).toSec());
	        myPlanner->setVelocityWithCloseLoopYawPitchOpenLoopDepth(0, 0, 0, myPlanner->getOpenLoopDepthSpeed(), frame);
	        loop_rate.sleep();
	        ros::spinOnce();
		}		

		//Start sending surge
		double gateTimeout = myPlanner -> getGateTimeout();
		ROS_INFO("Surge for %f seconds", gateTimeout);
		start_time = ros::Time::now();
		timeout = ros::Duration(gateTimeout);
		while (ros::Time::now() - start_time < timeout) {
			ROS_INFO_THROTTLE(1, "Sending surge velocity with close loop yaw and open loop depth. %f seconds left", (timeout - (ros::Time::now() - start_time)).toSec());
			myPlanner->setVelocityWithCloseLoopYawPitchOpenLoopDepth(myPlanner->getSurgeSpeed(), 0, 0, myPlanner->getOpenLoopDepthSpeed(), frame);
			loop_rate.sleep();
			ros::spinOnce();
		}
	} else {
		double closeLoopDepthTimeout = myPlanner -> getCloseLoopDepthTimeout();
		ROS_INFO("CLOSE LOOP DEPTH and close loop yaw and pitch for %f seconds", closeLoopDepthTimeout);
		start_time = ros::Time::now();
		timeout = ros::Duration(closeLoopDepthTimeout);
		while (ros::Time::now() - start_time < timeout) {
	        ROS_INFO_THROTTLE(1, "Sending depth, yaw, pitch first, before surge. %f seconds left", (timeout - (ros::Time::now() - start_time)).toSec());
			myPlanner->setVelocityWithCloseLoopYawPitchDepth(0, 0, 0, myPlanner->getCloseLoopDepth(), frame);
			loop_rate.sleep();
			ros::spinOnce();
		}

		double gateTimeout = myPlanner -> getGateTimeout();
		ROS_INFO("Surge for %f seconds", gateTimeout);
		start_time = ros::Time::now();
		timeout = ros::Duration(gateTimeout);
		while (ros::Time::now() - start_time < timeout) {
			ROS_INFO_THROTTLE(1, "Sending surge velocity with close loop yaw and depth. %f seconds left", (timeout - (ros::Time::now() - start_time)).toSec());
			myPlanner->setVelocityWithCloseLoopYawPitchDepth(myPlanner->getSurgeSpeed(), 0, 0, myPlanner->getCloseLoopDepth(), frame);
			loop_rate.sleep();
			ros::spinOnce();
		}
	}
	ROS_INFO("FINISHED THE GATE. Switching to lane");
	myPlanner->switchToTask(myPlanner->Lane);
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

	double myPoints[5] = {2.5, 0.0, 0.0, 0.0, myPlanner->getCloseLoopDepth()};
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
	frame = "/sensors/IMU_global_reference";

	ros::Rate loop_rate(50);
	loop_rate.sleep();

	myStatusUpdater->updateStatus(myStatusUpdater->gate3);
	ROS_INFO("Gate phase 3");
	loop_rate.sleep();

	ROS_INFO("Task_Gate::reached the front of the gate");
	//TODO: this phase doesn't really do anything anymore.
	//Controls requires continuous sending of velocity now.
	myPlanner->setVelocityWithCloseLoopYawPitchDepth(myPlanner->getSurgeSpeed(), 0, 0, myPlanner->getCloseLoopDepth(), frame);

	ROS_INFO("%s", "Task_Gate::The gate task has been completed.");
	loop_rate.sleep();

	//next task
	myPlanner->switchToTask(myPlanner->Lane);
}