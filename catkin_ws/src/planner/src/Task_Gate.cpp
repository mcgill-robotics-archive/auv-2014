#include "Task_Gate.h"

/**
 * Constructor.
 */
Task_Gate::Task_Gate() {
	id = "Gate Task";
}

/**
 * Destructor.
 */
Task_Gate::~Task_Gate() {

}

int Task_Gate::Execute() {
	ros::Rate loop_rate(50);
	loop_rate.sleep();
	
	ROS_INFO("%s", "Executing the gate task.");

	weAreHere("Beginning GATE task");
	loop_rate.sleep();

	ROS_INFO("%s", "Gate Task setting position relative to gate:  setPosition(-1, 0, 0, 0, 8.8)");

	setVisionObj(1);
	loop_rate.sleep();
	
	//std::vector<double> desired =  (-1.0, 0.0, 0.0, 0.0, 8.8);
	//double myPoints[] = {-1.0, 0.0, 0.0, 0.0, 8.8};
	//std::vector<double> desired (myPoints, myPoints + sizeof(myPoints) / sizeof(double) );
	double myPoints[5] = {-1.0, 0.0, 0.0, 0.0, 8.8};
	setTransform("/target/door");
	std::vector<double> desired(myPoints, myPoints + sizeof(myPoints) / sizeof(myPoints[0]));
	//desired = getTransform();

	while (!areWeThereYet_tf("/target/door")) {
		setPosition(desired);
		loop_rate.sleep();
	}
	
	ROS_INFO("%s", "The gate task has been completed.");

	weAreHere("Finishing GATE task");
	loop_rate.sleep();

	return 0;
}
