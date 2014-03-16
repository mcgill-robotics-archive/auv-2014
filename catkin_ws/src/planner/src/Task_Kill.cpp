#include "Task_Kill.h"

Task_Kill::Task_Kill() {
	id = "Task Kill";
}

int Task_Kill::Execute() {
	ros::Rate loop_rate(50);
	loop_rate.sleep();

	std::vector<double> desired_pos;
	
	//the section needs to be implemented so that we can resurface and set all 
	//messages to zero or default
	/*desired_pos.push_back(0.0);//
	desired_pos.push_back(0.0);
	desired_pos.push_back(0.0);
	desired_pos.push_back(0.0);
	desired_pos.push_back(0.0); 
	setPosition(desired_pos);*/
	setVelocity(0,0,0,2); //Sets x,y,yaw speeds all to zero and gives a depth of 0 i.e resurface

	std::cout<<"Mission completed"<<std::endl;
	weAreHere("RESURFACING");	
	loop_rate.sleep();

	return 0;
}
