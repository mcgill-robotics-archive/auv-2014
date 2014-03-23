//TaskFactory.hpp

// need to include Task.cpp, TaskConcrete.cpp, etc....
#ifndef Taskfactory_h
#define Taskfactory_h

#include <iostream>
#include <string>
#include <stdlib.h>
#include "TaskConcrete.hpp"
#include "Task.hpp"
#include "Task_Gate.h"
#include "Task_Lane.h"

#include "Task_Kill.h"

//#include "Config.hpp"
//#include "Task.cpp"

class TaskFactory{
 public:

 	TaskFactory();
 	~TaskFactory();

 	int parseTask(std::string id);
		
	int parseVersion(std::string id);

	Task* makeTask(std::string id);

	Task* getGateTask();

	Task* getEndTask();

};

#endif

