#ifndef TASK_H
#define TASK_H

#include <iostream>
#include "ros/ros.h"

class Task {
public:

	std::string id;

	Task();
	Task(std::string myid);

	virtual int Execute();

};

#endif
