#ifndef TASK_GATE_H
#define TASK_GATE_H

#include <string>
#include "Task.hpp"
#include "Interface.h"
#include <iostream>

class Task_Gate: public Task{
public:
	Task_Gate();
	int Execute();
};

#endif
