#ifndef TASK_KILL_H
#define TASK_KILL_H

#include <string>
#include "Task.hpp"
#include "Interface.h"
#include <iostream>

class Task_Kill: public Task{
public:
	Task_Kill();
	int Execute();
};

#endif
