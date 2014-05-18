#include "Task_Lane.h"

Task_Lane::Task_Lane(Planner* planner, StatusUpdater* mSU, int newPhase){
	myPlanner = planner;
	myStatusUpdater = mSU;
	phase = newPhase;
}

void Task_Lane::execute() {
	std::string frame = "/target/guideline";

	ros::Rate loop_rate(50);
	loop_rate.sleep();
	//look for the lane marker
	myPlanner->setVisionObj(2);
	loop_rate.sleep();
ROS_INFO("looking for the lane");
	tf::TransformListener listener;
	listener.waitForTransform(frame, "/robot/rotation_center",
			ros::Time(0), ros::Duration(300));
myPlanner->setVelocity(0, 0, 0, 8.8, frame);
ROS_INFO("found the lane");
	//position above the lane marker
	double myPoints[5] = {0.0, 0.0, 0.0, 0.0, 8.8};
	std::vector<double> desired(myPoints, myPoints + sizeof(myPoints) / sizeof(myPoints[0]));

	while (!myPlanner->areWeThereYet(frame, desired)) {
		ROS_DEBUG("Task_Lane::setPoints published");		
		myPlanner->setPosition(desired, frame);
		loop_rate.sleep();
	}

	//end the routine
	myPlanner->switchToTask(myPlanner->Kill);
}