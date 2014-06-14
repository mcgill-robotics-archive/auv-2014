#include "Lost_Lane.h"

Lost_Lane::Lost_Lane(Planner* planner, StatusUpdater* mSU, int newPhase){
	myPlanner = planner;
	myStatusUpdater = mSU;
	phase = newPhase;
}

void Lost_Lane::execute() {
	switch(phase) {
		case 1:
			phase1();
			break;
		case 2:

			break;
		case 3:

			break;
	}
}

void Lost_Lane::phase1() {
}