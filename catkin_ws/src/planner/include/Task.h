#ifndef Task_h
#define Task_h

#include "Planner.h"
#include "StatusUpdater.h"

class Task{
	public:
		virtual void execute ();
		Planner* myPlanner;
		Task(Planner* planner, StatusUpdater* mSU);
		Task();
		StatusUpdater* myStatusUpdater;

	private:
};

#endif