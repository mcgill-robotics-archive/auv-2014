#include "Task_Lane.h"

Task_Lane::Task_Lane(Planner* planner, StatusUpdater* mSU, int newPhase){
	myPlanner = planner;
	myStatusUpdater = mSU;
	phase = newPhase;
}

void Task_Lane::execute() {
	std::string imuFrame = "/sensors/IMU_global_reference";
	std::string frame = "/target/lane";

	ros::Rate loop_rate(50);
	loop_rate.sleep();

	myStatusUpdater->updateStatus(myStatusUpdater->lane1);

	ROS_INFO("Started Lane Task");

	loop_rate.sleep();

	if (myPlanner -> getUseCvForLane()) {
			//look for the lane marker
		myPlanner->setVisionObj(2);
		ROS_INFO("looking for the lane");
	    // Move straight until we detect something
	    while (!myPlanner->getSeeObject()) {
			myPlanner->setVelocityWithCloseLoopYawPitchDepth(myPlanner->getSurgeSpeed(), 0, 0, myPlanner->getCloseLoopDepth(), imuFrame);
	        loop_rate.sleep();
	    }
		ROS_INFO("FOUND THE LANE. Going to wait for transform");

		tf::TransformListener listener;
		listener.waitForTransform(frame, "/robot/rotation_center",
				ros::Time(0), ros::Duration(0.4));
	 
	    ROS_INFO("Got the transform");
		myStatusUpdater->updateStatus(myStatusUpdater->lane2);
		loop_rate.sleep();


		// From here we can turn right now (right after we detected the Lane), or
		// wait for the distance threshold to be reached.
	    if (myPlanner->getUseDistanceToLaneThreshold()) {
	    	ROS_INFO("Going to use distance to lane threshold");
	    	// While our relative distance with the Lane is bigger than the threshold continue moving forward.
	    	while(myPlanner->getCurrentX(imuFrame) > myPlanner->getRelativeDistanceToLaneThreshold()) {
	    		myPlanner->setVelocityWithCloseLoopYawPitchDepth(myPlanner->getSurgeSpeed(), 0, 0, myPlanner->getCloseLoopDepth(), imuFrame);
	        	loop_rate.sleep();
	    	}
	    }
	}

	// Commented out this so that we can use an hardcoded relative angle
	// double myPoints[5] = {0.0, 0.0, 0.0, 0.0, myPlanner->getCloseLoopDepth()};

	// From here we can either take the value given by cv or the hardcoded one:
	
	if (myPlanner->getUseHardcodedLaneAngleAfterGate()) {
		double yawTimeout = myPlanner -> getYawTimeout();
		stop(imuFrame, yawTimeout); //stop for a few seconds

		double hardcodedYaw = myPlanner->getHardcodedRelativeLaneAngleAfterGate();

		ROS_INFO("Using hardcoded yaw for lane: %f radians", hardcodedYaw);

		ROS_INFO("Sending Yaw = %f for %f seconds", hardcodedYaw, yawTimeout);
		ros::Time start_time = ros::Time::now();
		ros::Duration timeout(yawTimeout);
		while(ros::Time::now() - start_time < timeout) {
	        ROS_INFO_THROTTLE(1, "Sending yaw = %f", hardcodedYaw);
	        myPlanner->setVelocityWithCloseLoopYawPitchDepth(0, hardcodedYaw, 0, myPlanner->getCloseLoopDepth(), imuFrame);
	        loop_rate.sleep();
	        ros::spinOnce();
		}

	} else { //use relative angle from CV
		ROS_INFO("Using relative angle from CV");

		double myPoints[5];
		myPoints[0] = 0.0;
		myPoints[1] = 0.0;
		myPoints[2] = 0.0;
		myPoints[3] = 0.0;
		myPoints[4] = myPlanner->getCloseLoopDepth();

		std::vector<double> desired(myPoints, myPoints + sizeof(myPoints) / sizeof(myPoints[0]));

		// Rotate the robot to the desired angle
		while (!myPlanner->areWeThereYet(frame, desired)) {
			ROS_DEBUG("Task_Lane::setPoints published");		
			myPlanner->setPosition(desired, frame);
			loop_rate.sleep();
		}
	}

	ROS_INFO("ALIGNED WITH LANE.");
    
    flash();
	myStatusUpdater->updateStatus(myStatusUpdater->lane3);
	loop_rate.sleep();

	
	double laneTimeout = myPlanner -> getLaneTimeout();
	goStraightFromCurrentPosition(imuFrame, myPlanner->getHardcodedRelativeLaneAngleAfterGate(), laneTimeout);

	//stop and turn after first lane
	// stopAndTurn(imuFrame, 0.087);
	// goStraightFromCurrentPosition(imuFrame, 20);

	// We have the possibility of executing the hydrophones task after the Lanet task is completed.
	if (myPlanner->getDoHydrophonesAfterLane()) {
		ROS_INFO("DOING HYDROPHONES TASK");
		if (myPlanner-> getUsingTimerForHydrophonesInsteadOfDistance()) {
			ROS_INFO("EXTRA AUTONOMOUS HYDROPHONES");
			double pingerYaw = myPlanner -> getEstimatedRelativeAngleForThePinger();
			stopAndTurn(imuFrame, pingerYaw);
			goStraightFromCurrentPosition(imuFrame, pingerYaw, myPlanner->getTimerForHydrophonesTask());
			flash();
			resurface(imuFrame);
		} else {
			ROS_INFO("REGULAR AUTONOMOUS HYDROPHONES");
			myPlanner -> startHydrophones();
		}
	} else {
		ROS_INFO("NOT DOING HYDROPHONES, RESURFACING");
		flash();
		resurface(imuFrame);
	}
}

void Task_Lane::goStraightFromCurrentPosition(std::string frame, double yaw, double timeout) {
	ROS_INFO("Going forward from current position using frame %s", frame.c_str());

	ros::Rate loop_rate(50);
;
	ROS_INFO("Follow lane using frame %s for %f seconds", frame.c_str(), timeout);
	ros::Time start_time = ros::Time::now();
	ros::Duration timeoutDuration(timeout);
	while(ros::Time::now() - start_time < timeoutDuration) {
	        ROS_INFO_THROTTLE(1, "Sending yaw = %f, pitch = %f, depth = %f", yaw, 0.0, myPlanner->getCloseLoopDepth());
			myPlanner->setVelocityWithCloseLoopYawPitchDepth(myPlanner->getSurgeSpeed(), yaw, 0, myPlanner->getCloseLoopDepth(), frame);
	        loop_rate.sleep();
	        ros::spinOnce();
	}

}

//TODO: use get current angle
void Task_Lane::stop(std::string frame, double timeout) {
	ROS_INFO("Stopping");

	ros::Rate loop_rate(50);

	ROS_INFO("Stopping using frame %s for %f seconds", frame.c_str(), timeout);
	ros::Time start_time = ros::Time::now();
	ros::Duration timeoutDuration(timeout);
	while(ros::Time::now() - start_time < timeoutDuration) {
	        ROS_INFO_THROTTLE(1, "Sending yaw = %f, pitch = %f, depth = %f", 0.0, 0.0, myPlanner->getCloseLoopDepth());
			myPlanner->setVelocity(0.0, 0.0, 0.0, myPlanner->getCloseLoopDepth(), frame);
	        loop_rate.sleep();
	        ros::spinOnce();
	}
	ROS_INFO("Stopped");
}

void Task_Lane::stopAndTurn(std::string frame, double yaw) {
	ros::Rate loop_rate(50);

	double yawTimeout = myPlanner -> getYawTimeout();
	stop(frame, yawTimeout); //stop for a few seconds

	ROS_INFO("Sending Yaw = %f for %f seconds", yaw, yawTimeout);
	ros::Time start_time = ros::Time::now();
	ros::Duration timeoutDuration(yawTimeout);
	while(ros::Time::now() - start_time < timeoutDuration) {
        ROS_INFO_THROTTLE(1, "Sending yaw = %f", yaw);
        myPlanner->setVelocityWithCloseLoopYawPitchDepth(0, yaw, 0, myPlanner->getCloseLoopDepth(), frame);
        loop_rate.sleep();
        ros::spinOnce();
	}
}

void Task_Lane::flash() {
	ros::Rate loop_rate(50);
	myStatusUpdater->updateStatus(myStatusUpdater->flash1);
	loop_rate.sleep();
	myStatusUpdater->updateStatus(myStatusUpdater->flash2);
	loop_rate.sleep();
	myStatusUpdater->updateStatus(myStatusUpdater->flash1);
	loop_rate.sleep();
	myStatusUpdater->updateStatus(myStatusUpdater->flash2);
	loop_rate.sleep();
	myStatusUpdater->updateStatus(myStatusUpdater->flash1);
	loop_rate.sleep();
}

void Task_Lane::resurface(std::string frame) {
	ros::Rate loop_rate(50);
	double openLoopDepthTimeout = myPlanner -> getOpenLoopDepthTimeout();
	ROS_INFO("OPEN LOOP DEPTH and close loop yaw and pitch for %f seconds", openLoopDepthTimeout);
	ros::Time start_time = ros::Time::now();
	ros::Duration timeoutOL =ros::Duration(openLoopDepthTimeout);
	while(ros::Time::now() - start_time < timeoutOL) {
        ROS_INFO_THROTTLE(1, "Sending depth, yaw, pitch first, before surge. %f seconds left", (timeoutOL - (ros::Time::now() - start_time)).toSec());
        myPlanner->setVelocityWithCloseLoopYawPitchOpenLoopDepth(0, 0, 0, -1 * myPlanner->getOpenLoopDepthSpeed(), frame);
        loop_rate.sleep();
        ros::spinOnce();
	}
}