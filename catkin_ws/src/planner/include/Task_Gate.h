#ifndef Task_Gate_h
#define Task_Gate_h

#include "Task.h"

class Task_Gate: public Task{
public:
	Task_Gate(Planner* planner, StatusUpdater* mSU);
	void execute();
};

#endif