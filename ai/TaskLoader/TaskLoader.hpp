//TaskLoader.hpp

// need to include Task.cpp, TaskConcrete.cpp, etc....
#include <iostream>
#include <string>
#include "quick_parse.hpp"
#include "TaskConcrete.cpp"
#include "Task.cpp"

class TaskLoader
{
public:
	TaskLoader();
	//int parseTask(std::string x);
	//int parseVersion(std::string x);
	void makeTask(std::string x);
private:
	int t;
	int v;
};