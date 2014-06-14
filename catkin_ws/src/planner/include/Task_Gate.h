#ifndef Task_Gate_h
#define Task_Gate_h

#include "Task.h"

class Task_Gate: public Task{
	public:
		Task_Gate(Planner* planner, StatusUpdater* mSU, int newPhase);
		void execute();
	private:
		void phase1();
		void phase2();
		void phase3();
		std::string frame;
};

#endif