#include "Invoker.h"

Invoker::Invoker(){

}

void Invoker::AddTask(Task * t){
	TaskList.push_back(t);
}

void Invoker::StartRun(){
	int i;
	for(i = 0; i < TaskList.size(); i++){
		std::cout<<"Starting task: " << TaskList.at(i)->id <<std::endl;		
		TaskList.at(i)->Execute();
	}

}
