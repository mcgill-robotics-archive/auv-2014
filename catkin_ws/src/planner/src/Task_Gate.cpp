#include "Task_Gate.h"

Task_Gate::Task_Gate(Planner* planner, StatusUpdater* mSU, int newPhase){
	myPlanner = planner;
	myStatusUpdater = mSU;
	phase = newPhase;
}

void Task_Gate::execute() {
	//setup stuff that should always run
	frame = "/target/gate";
	ROS_INFO("%s", "Executing the gate task.");

	if(phase <= 1) {
		phase1();
	}
	if(phase <= 2) {
		phase2();
	}
	//etc
}

void Task_Gate::phase1() {
	ros::Rate loop_rate(50);
	loop_rate.sleep();
	myPlanner->setVisionObj(0);
	loop_rate.sleep();
	myPlanner->setVisionObj(1);
	loop_rate.sleep();
	myStatusUpdater->updateStatus(myStatusUpdater->gate1);
	loop_rate.sleep();
		myPlanner->setVelocity(2, 0, 0, 8.8, frame);
		loop_rate.sleep();
		tf::TransformListener listener;
	listener.waitForTransform(frame, "/robot/rotation_center",
			ros::Time(0), ros::Duration(300));
ROS_INFO("we found it");
	myStatusUpdater->updateStatus(myStatusUpdater->gate2);
	loop_rate.sleep();
	
	double myPoints[5] = {1.0, 0.0, 0.0, 0.0, 8.8};
	std::vector<double> desired(myPoints, myPoints + sizeof(myPoints) / sizeof(myPoints[0]));
	
	while (!myPlanner->areWeThereYet(frame, desired)) {
		ROS_DEBUG("Task_Gate::setPoints published");		
		myPlanner->setPosition(desired, frame);
		loop_rate.sleep();
	}	
}

void Task_Gate::phase2() {
	ros::Rate loop_rate(50);
	loop_rate.sleep();

	ROS_INFO("Task_Gate::reached the front of the gate");
	myPlanner->setVelocity(2, 0, 0, 8.8, frame);

	ROS_INFO("%s", "Task_Gate::The gate task has been completed.");
	myStatusUpdater->updateStatus(myStatusUpdater->gate3);
	loop_rate.sleep();

	//next task
	myPlanner->switchToTask(myPlanner->Lane);
}