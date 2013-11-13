#ifndef Task_h
#define Task_h

#include "Logger.h"
#include "Map.h"
#include "ROS.h"
#include <string>


class Task {

 public:
	task(std::string task_name, ROS ros, Logger log, Map m ){
		TaskName = task_name;
		Ros = ros;
		logger = log;
		map = m;
		// logger log -> created the 
	}
    virtual void Execute();

 private:
    ROS Ros;
    Logger logger;
    Map map;
	std::string TaskName;
};

#endif // Task_h
