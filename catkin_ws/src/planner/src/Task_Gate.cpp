#include "Task_Gate.h"

Task_Gate::Task_Gate(Planner* planner, StatusUpdater* mSU){
	myPlanner = planner;
	myStatusUpdater = mSU;
}

void Task_Gate::execute() {
	ros::Rate loop_rate(50);
	loop_rate.sleep();
	std::string frame = "/target/gate";
	ROS_INFO("%s", "Executing the gate task.");

	myStatusUpdater->updateStatus(myStatusUpdater->startingGate);
	loop_rate.sleep();

	myPlanner->setVisionObj(1);
	loop_rate.sleep();
	
	//std::vector<double> desired =  (1.0, 0.0, 0.0, 0.0, 8.8);
	//double myPoints[] = {-1.0, 0.0, 0.0, 0.0, 8.8};
	//std::vector<double> desired (myPoints, myPoints + sizeof(myPoints) / sizeof(double) );
	double myPoints[5] = {1.0, 0.0, 0.0, 0.0, 8.8};
	std::vector<double> desired(myPoints, myPoints + sizeof(myPoints) / sizeof(myPoints[0]));
	//desired = getTransform();

	myStatusUpdater->updateStatus(myStatusUpdater->reachedGate);
	loop_rate.sleep();
	
	while (!myPlanner->areWeThereYet(frame, desired)) {
		ROS_INFO("Task_Gate::setPoints published");		
		myPlanner->setPosition(desired, frame);
		loop_rate.sleep();
	}
	ROS_INFO("Task_Gate::reached the front of the gate");
	double motorBoat[4] = {5.0, 0.0, 0.0, 8.8};
	//setTransform("/target/gate");
	std::vector<double> motor(motorBoat, motorBoat + sizeof(motorBoat) / sizeof(motorBoat[0]));
	myPlanner->setVelocity(5, 0, 0, 8.8, frame);

	ROS_INFO("%s", "Task_Gate::The gate task has been completed.");
	myStatusUpdater->updateStatus(myStatusUpdater->completedGate);
	loop_rate.sleep();

	//look for the lane marker
	myPlanner->setVisionObj(2);
}