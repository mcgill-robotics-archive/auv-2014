#include "Task_Lane.h"

Task_Lane::Task_Lane() {
	id = "Lane Task";
}

int Task_Lane::Execute() {
	ros::Rate loop_rate(50);
	loop_rate.sleep();

	std::cout << ">> Executing Lane Task" << std::endl;
	weAreHere("Beginning LANE task");	loop_rate.sleep();
/*
	std::cout<<"Lane Task finding lane marker"<<std::endl;
	setVisionObj(2);	loop_rate.sleep();
	setVelocity(0.0, 0.0, 0.0, 3);	loop_rate.sleep();
	rosSleep(30);

	std::cout<<"Lane Task correcting angle over lane marker"<<std::endl;
	setPosition(0.0, 0.0, 0.0, 0.0, 5); loop_rate.sleep();
	weAreHere("Floating above lane marker"); loop_rate.sleep();
	rosSleep(30);

	std::cout<<"Lane Task going where no lane task has gone before"<<std::endl;
	setVelocity(20.0, 0.0, 0.0, 5); loop_rate.sleep();
*/
	std::cout<<"Lane Task completed"<<std::endl;
	weAreHere("Finishing LANE task");	loop_rate.sleep();

	return 0;
}
