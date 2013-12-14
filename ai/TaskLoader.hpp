//TaskLoader.hpp

// need to include Task.cpp, TaskConcrete.cpp, etc....
#ifndef TASKLOADER_H
#define TASKLOADER_H

#include <iostream>
#include <string>
#include "quick_parse.hpp"
#include "TaskConcrete.hpp"
//#include "Task.cpp"

	Task* makeTask(std::string id);

#endif

