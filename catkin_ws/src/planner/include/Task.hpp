#ifndef TASK_H
#define TASK_H
#include <iostream>
#include "Interface.h"

class Task{
public:
	// Task(std::string task_name, ROS ros, Logger log){ //Map m ){
	// 	TaskName = task_name;
	// 	Ros = ros;
	// 	logger = log;
	// 	//map = m;
	// 	// logger log -> created the 
	// }
	std::string id;
	Interface* cur_interface;

	Task(Interface* inf);
	Task(std::string myid);

	virtual int Execute();

};

#endif