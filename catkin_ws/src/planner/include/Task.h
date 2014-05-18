#ifndef Task_h
#define Task_h

#include "Planner.h"
#include "StatusUpdater.h"

class Task{
	public:
		virtual void execute ();
		Task(Planner* planner, StatusUpdater* mSU, int newPhase);
		Task();
		StatusUpdater* myStatusUpdater;
		Planner* myPlanner;
		int phase;

	private:
};

#endif