//TaskConcrete.cpp

#include "TaskConcrete.hpp"

//add implementaion of the tasks here don't forget to include the 
//header in the header file also include the task choice in the task factory.


TorpedoTask::TorpedoTask(){
	id = "Rocket_Task";
}
int TorpedoTask::Execute(){
	std::cout <<">> Shoot Torpedo!!!"<< std::endl;
	return 0;
}

Task_1v1::Task_1v1(){
	id = "Rocket_Task";
}
int Task_1v1::Execute()
{
	std::cout<< ">> 1.1: Move Right, then fire Missiles!" << std::endl;
	return 0;
}

Task_1v2::Task_1v2(){
	id = "Rocket_Task";
}
int Task_1v2::Execute()
{
	std::cout <<">> 1.2: Move Left, then fire Missiles!" << std::endl;
	return 0;
}

Task_2v1::Task_2v1(){
	id = "Rocket_Task";
}
int Task_2v1::Execute()
{
	std::cout<< ">> 2.1: Move Straight, then fire Missiles!" << std::endl;
	return 0;
}


/*int main(){
	Task* MyTask = new Task_1v3();
	MyTask->execute();
return 0;
}*/
