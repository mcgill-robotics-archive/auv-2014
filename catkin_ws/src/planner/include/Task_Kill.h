#ifndef Task_Kill_h
#define Task_Kill_h

#include "Task.h"

class Task_Kill: public Task{
public:
	Task_Kill(Planner* planner, StatusUpdater* mSU);
	void execute();
};

#endif