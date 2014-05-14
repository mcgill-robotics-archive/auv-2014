#ifndef Task_Lane_h
#define Task_Lane_h

#include "Task.h"

class Task_Lane: public Task{
public:
	Task_Lane(Planner* planner, StatusUpdater* mSU);
	void execute();
};

#endif