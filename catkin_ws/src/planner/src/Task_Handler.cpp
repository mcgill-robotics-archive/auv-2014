#include "Task_Handler.h"

void run_routine(int start_task, int end_task) {
	for(int i = start_task; i <= end_task; i++) {
		switch(i) {
			case 1:
			ROS_INFO("Planner::Task_Handler - beginning gate task");
				run_gate();
			break;

			case 2:
			ROS_INFO("Planner::Task_Handler - beginning lane task");
				run_lane();
			break;

			case 3:
			ROS_INFO("Planner::Task_Handler - beginning buoy task");
				run_buoy();
			break;
		}
	}
	
	ROS_INFO("Planner::Task_Handler - ending routine");
	end_routine();
}

//copy of Task_Gate, will clean up eventually
int run_gate() {
	ros::Rate loop_rate(50);
	loop_rate.sleep();
	std::string frame = "/target/gate";
	ROS_INFO("%s", "Executing the gate task.");

	weAreHere("Beginning GATE task");
	loop_rate.sleep();

	setVisionObj(1);
	updateBlinkyTape(PURPS);
	loop_rate.sleep();
	
	//std::vector<double> desired =  (1.0, 0.0, 0.0, 0.0, 8.8);
	//double myPoints[] = {-1.0, 0.0, 0.0, 0.0, 8.8};
	//std::vector<double> desired (myPoints, myPoints + sizeof(myPoints) / sizeof(double) );
	double myPoints[5] = {1.0, 0.0, 0.0, 0.0, 8.8};
	setTransform("/target/gate");
	std::vector<double> desired(myPoints, myPoints + sizeof(myPoints) / sizeof(myPoints[0]));
	//desired = getTransform();

	ROS_INFO("Task_Gate::reached task");
	while (!areWeThereYet_tf(frame, desired)) {
	ROS_INFO("Task_Gate::setPoints published");		
		setPosition(desired, frame);

		loop_rate.sleep();
	}
	ROS_INFO("Task_Gate::reached the front of the gate");
	double motorBoat[4] = {5.0, 0.0, 0.0, 8.8};
	//setTransform("/target/gate");
	std::vector<double> motor(motorBoat, motorBoat + sizeof(motorBoat) / sizeof(motorBoat[0]));
	setVelocity(5, 0, 0, 8.8, frame);
	setVisionObj(2);
	ROS_INFO("%s", "Task_Gate::The gate task has been completed.");

	weAreHere("Finishing GATE task");
	loop_rate.sleep();

	return 0;
}

int run_lane() {
	return 0;
}

int run_buoy() {
	return 0;
}

int end_routine() {
	ros::Rate loop_rate(50);
	loop_rate.sleep();

	double nullVals[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	setPoints(nullVals, "/robot/rotation_center");
	loop_rate.sleep();
	
	weAreHere("RESURFACING");
	loop_rate.sleep();
	
	return 0;
}
