#include "Task.h" 

Task::Task(Planner* planner, StatusUpdater* mSU, int newPhase) {
	myPlanner = planner;
	myStatusUpdater = mSU;
	phase = newPhase;
}

Task::Task(){}

void Task::execute() {
	std::cout<<"oops";
}