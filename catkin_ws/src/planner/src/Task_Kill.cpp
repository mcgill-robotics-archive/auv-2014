#include "Task_Kill.h"

Task_Kill::Task_Kill() {
	id = "Task Kill";
}

int Task_Kill::Execute() {
	ros::Rate loop_rate(50);
	loop_rate.sleep();
	
	//the section needs to be implemented so that we can resurface and set all 
	//messages to zero or default


	std::cout<<"Mission completed"<<std::endl;
	weAreHere("RESURFACING");	
	loop_rate.sleep();

	return 0;
}
