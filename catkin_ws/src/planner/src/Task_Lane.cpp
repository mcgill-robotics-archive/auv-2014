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

    // Move straight until we detect something
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
	
	// From here we can turn right now (right after we detected the line), or
	// wait for the distance threshold to be reached.
    if (myPlanner->getUseDistanceWithLineThreshold()) {
    	// While our relative distance with the line is bigger than the threshold continue moving forward.
    	// TODO (ejeadry) get the relative distance with the line and wait for the threshold to be reached

    	// while() {
    	// 	// wait for the distance to be smaller than the specified threshold
    	// }
    }

	// Commented out this so that we can use an hardcoded relative angle
	// double myPoints[5] = {0.0, 0.0, 0.0, 0.0, myPlanner->getCloseLoopDepth()};

	// From here we can either take the value given by cv or the hardcoded one:
	double myPoints[5];
	if (myPlanner->getUseHardcodedLineAngleAfterGate()) {
		// TODO (ejeadry) set the desired relative yaw to be the one set in 'hardcodedRelativeLineAngleAfterGate'
		myPoints[0] = 0.0;
		myPoints[1] = 0.0;
		myPoints[2] = 0.0;
		myPoints[3] = 0.0;
		myPoints[4] = myPlanner->getCloseLoopDepth();
	} else {
		myPoints[0] = 0.0;
		myPoints[1] = 0.0;
		myPoints[2] = 0.0;
		myPoints[3] = 0.0;
		myPoints[4] = myPlanner->getCloseLoopDepth();
	}

	std::vector<double> desired(myPoints, myPoints + sizeof(myPoints) / sizeof(myPoints[0]));


	// Rotate the robot to the desired angle
	while (!myPlanner->areWeThereYet(frame, desired)) {
		ROS_DEBUG("Task_Lane::setPoints published");		
		myPlanner->setPosition(desired, frame);
		loop_rate.sleep();
	}
	myStatusUpdater->updateStatus(myStatusUpdater->lane3);
	loop_rate.sleep();

	goStraightFromCurrentPosition(frame);
}

void Task_Lane::goStraightFromCurrentPosition(std::string frame) {
	ros::Rate loop_rate(50);

	geometry_msgs::PoseStamped pose = myPlanner->getRelativePose(frame);
	double x = pose.pose.orientation.x;
	double y = pose.pose.orientation.y;
	double z = pose.pose.orientation.z;
	double w = pose.pose.orientation.w;
	double pitch = 1
			* -atan(
					(2.0f * (x * z + w * y))
							/ sqrt(
									1.0f
											- pow((2.0f * x * z + 2.0f * w * y),
													2.0f))); // multiply by 57.2957795130823f to convert to degrees
	double yaw = 1
			* atan2(2.0f * (x * y - w * z), 2.0f * w * w - 1.0f + 2.0f * x * x);

	double laneTimeout = myPlanner -> getLaneTimeout();
	ROS_INFO("Follow lane using frame %s for %f seconds", frame.c_str(), laneTimeout);
	ros::Time start_time = ros::Time::now();
	ros::Duration timeout(laneTimeout);
	while(ros::Time::now() - start_time < timeout) {
	        ROS_INFO_THROTTLE(1, "Sending yaw = %f, pitch = %f, depth = %f", yaw, pitch, myPlanner->getCloseLoopDepth());
			myPlanner->setVelocityWithCloseLoopYawPitchDepth(myPlanner->getSurgeSpeed(), yaw, pitch, myPlanner->getCloseLoopDepth(), frame);
	        loop_rate.sleep();
	        ros::spinOnce();
	}

	// We have the possibility of executing the hydrophones task after the linet task is completed.
	if (myPlanner->getDoHydrophonesAfterLine()) {
		// TODO (ejeadry) Implement the hydrophones task
		double estimatedRelativeAngleOfPinger = myPlanner->getEstimatedRelativeAngleForThePinger();


		// TODO (ejeadry) set the angle to the approximated pinger position
	} else {
		// TODO (ejeadry) If we are not doing the hydrophones then we should stop the robot after the timeout period
	}
}