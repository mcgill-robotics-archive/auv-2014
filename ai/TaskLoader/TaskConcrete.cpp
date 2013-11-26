//TaskConcrete.cpp
#include <iostream>
#include "TaskConcrete.hpp"
#include <string>
#include "Task.cpp"

class Task_1v1 : public Task
{
	public: 

		 void execute()
		 {
		 	std::cout<< "1.1: Move Right, then fire Missiles!" << std::endl;
		 }
};

class Task_1v2 : public Task
{
	public: 

		 void execute()
		 {
		 	std::cout<< "1.2: Move Left, then fire Missiles!" << std::endl;
		 }
};

class Task_1v3 : public Task
{
	public: 

		 void execute()
		 {
		 	std::cout<< "1.3: Move Straight, then fire Missiles!" << std::endl;
		 }
};

int main(){
	Task* MyTask = new Task_1v3();
	MyTask->execute();
return 0;
}