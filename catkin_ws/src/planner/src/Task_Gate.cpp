#include "Task_Gate.h"

Task_Gate::Task_Gate() {
	id = "Gate Task";
}

int Task_Gate::Execute() {
	ros::Rate loop_rate(50);
	loop_rate.sleep();
	
	std::cout << ">> Executing Gate Task" << std::endl;
	weAreHere("Beginning GATE task");	loop_rate.sleep();

	std::cout<<"Gate Task setting position relative to gate:  setPosition(-1, 0, 0, 0, 0, 8.8) "<<std::endl;
	setVisionObj(1);	loop_rate.sleep();
	
	/*once simulator and controls work:
	
	add method:
	while (currentPos != desiredPos) {
		setPosition(desiredPos)
	}
	
	
	*/
	setPosition(-1.0, 0.0, 0.0, 0.0, 8.8);	loop_rate.sleep();

	std::cout<<"Gate Task completed"<<std::endl;
	weAreHere("Finishing GATE task");	loop_rate.sleep();

	return 0;
}
