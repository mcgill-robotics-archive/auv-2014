#include "Task.h" 

Task::Task(Planner* planner, StatusUpdater* mSU) {
	myPlanner = planner;
	myStatusUpdater = mSU;
}

Task::Task(){}

void Task::execute() {
	std::cout<<"oops";
}