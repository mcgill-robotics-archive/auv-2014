#ifndef Task_Lane_h
#define Task_Lane_h

#include "Task.h"

class Task_Lane: public Task{
	public:
		Task_Lane(Planner* planner, StatusUpdater* mSU, int newPhase);
		void execute();
	private:
		void goStraightFromCurrentPosition(std::string frame, double yaw, double timeout);
		void stop(std::string frame, double timeout);
		void stopAndTurn(std::string frame, double yaw);
		void flash();
		void resurface(std::string frame);
};

#endif