#ifndef TASK_GATE_H
#define TASK_GATE_H

#include <string>
#include "Task.hpp"
#include "Interface.h"
#include <iostream>
#include <vector>

class Task_Gate: public Task{
public:
	Task_Gate();
	~Task_Gate();
	int Execute();
};

#endif
