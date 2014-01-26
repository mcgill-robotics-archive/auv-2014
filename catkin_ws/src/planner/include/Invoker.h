
#ifndef Invoker_h
#define Invoker_h

#include <iostream>
#include <vector>
#include "Task.hpp"
 
//#include "Logger.h"
//#include "Map.h"
//#include "ROS.h"

class Invoker{

/**
* functions to be called in Invoker.h 
*/
 public:
 	Invoker();
 	void AddTask(Task * t);
 	void StartRun();
 private:
 	std::vector<Task *> TaskList;

};

#endif // Task_h
