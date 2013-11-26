//TaskConcrete.hpp
#ifndef TASKCONCRETE_H
#define TASKCONCRETE_H
#include <string>
#include "Task.hpp"
#include <iostream>

class Task_1v1 : public Task{
public:
	void execute();
};

class Task_1v2 : public Task{
public:
	void execute();
};
class Task_1v3 : public Task{
public:
	void execute();
};

#endif