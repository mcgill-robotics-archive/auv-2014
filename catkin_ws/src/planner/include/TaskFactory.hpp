//TaskLoader.hpp

// need to include Task.cpp, TaskConcrete.cpp, etc....
#ifndef Taskloader_h
#define Taskloader_h

#include <iostream>
#include <string>
#include "quick_parse.hpp"
#include "TaskConcrete.hpp"
#include "Interface.h"
//#include "Config.hpp"

//#include "Task.cpp"
class TaskFactory{
 public:

 	TaskFactory(Interface* inf);
	Task* makeTask(std::string id);

	Interface* inf;

};

#endif

