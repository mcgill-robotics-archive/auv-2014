
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
 	void StartAt(int n);
 private:
 	std::vector<Task *> TaskList;


};


/*************INCLUDE AT BOTTOM*********************/
/*BECAUSE OF ROUND ABOUT INCLUDES FROM INTERFACE.H TO INVOKER.H*/
#include "Interface.h"

#endif // Task_h
