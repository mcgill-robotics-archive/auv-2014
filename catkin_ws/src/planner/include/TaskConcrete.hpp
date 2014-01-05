//TaskConcrete.hpp
#ifndef TASKCONCRETE_H
#define TASKCONCRETE_H
#include <string>
#include "Task.hpp"
#include <iostream>

class TorpedoTask: public Task{
public:
	TorpedoTask();
	int Execute();
};

class Task_1v1 : public Task{
public:
	Task_1v1();
	int Execute();
};

class Task_1v2 : public Task{
public:
	Task_1v2();
	int Execute();
};
class Task_2v1 : public Task{
public:
	Task_2v1();
	int Execute();
};

#endif