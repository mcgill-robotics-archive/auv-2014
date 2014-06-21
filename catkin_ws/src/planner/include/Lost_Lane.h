#ifndef Lost_Lane_h
#define Lost_Lane_h

#include "Task.h"

class Lost_Lane: public Task{
	public:
		Lost_Lane(Planner* planner, StatusUpdater* mSU, int newPhase);
		void execute();

	private:
		void phase1();
		double xSpeed;	
};

#endif