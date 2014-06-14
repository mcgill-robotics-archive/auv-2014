#ifndef Lost_Gate_h
#define Lost_Gate_h

#include "Task.h"

class Lost_Gate: public Task{
	public:
		Lost_Gate(Planner* planner, StatusUpdater* mSU, int newPhase);
		void execute();

	private:
		void phase1();	
};

#endif