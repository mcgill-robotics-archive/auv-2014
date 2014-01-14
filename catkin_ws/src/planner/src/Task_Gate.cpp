#include "Task_Gate.h"


Task_Gate::Task_Gate(){
	id = "Gate Task";
}

int Task_Gate::Execute(){
	std::cout << "Executing Gate Task" << std::endl;
	
	std::cout<<"Gate Task setting velocity:   setVelocity(1, 1, 1, 1) "<<std::endl;
	setVelocity(1, 1, 1, 1);	
	
	std::cout<<"Gate Task setting position:   setPosition(1.0, 2.0, 3.0, 4.0, 5.0);"<<std::endl;
	setPosition(1.0, 2.0, 3.0, 4.0, 5.0);
	
	std::cout<<"Gate Task starting ps3 control"<<std::endl<<"..."<<std::endl;


	int i = 0;
	ros::Rate loop_rate(10);
	while(i < 10){
		ps3Control(); 
		ros::spinOnce();
		loop_rate.sleep();
		i++;
	}

	return 0;
}

