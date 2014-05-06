#ifndef Task_h
#define Task_h

#include "Planner.h"
#include "StatusUpdater.h"

class Task{
	public:
		void execute ();
		Planner* myPlanner;
		Task(Planner* planner, StatusUpdater* mSU);

	private:
		StatusUpdater* myStatusUpdater;
};

#endif