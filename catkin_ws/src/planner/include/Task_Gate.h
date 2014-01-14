#include <string>
#include "Task.hpp"
#include "Interface.h"
#include <iostream>

#ifndef TASK_GATE_H
#define TASK_GATE_H


class Task_Gate: public Task{
public:
	Task_Gate();
	int Execute();
};

#endif
