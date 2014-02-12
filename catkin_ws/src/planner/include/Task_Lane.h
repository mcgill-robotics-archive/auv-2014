#ifndef TASK_LANE_H
#define TASK_LANE_H

#include <string>
#include "Task.hpp"
#include "Interface.h"
#include <iostream>

class Task_Lane: public Task{
public:
	Task_Lane();
	int Execute();
};

#endif
