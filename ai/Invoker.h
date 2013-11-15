#ifndef Invoker_h
#define Invoker_h

#include "Task.h"
#include "Logger.h"
//#include "Map.h"
//#include "ROS.h"



class Invoker{

 public:
 	void AddTask(Task * t);
 	void StartRun();
 private:
 	std::vector<Task *> TaskList;

};

#endif // Task_h
