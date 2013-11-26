//TaskLoader.hpp

// need to include Task.cpp, TaskConcrete.cpp, etc....
#include <iostream>
#include <string>

class TaskLoader
{
public:
	TaskLoader();
	int parseTask(std::string x);
	int parseVersion(std::string x);
	Task* makeTask(std::string x);
private:
	int t;
	int v;
};