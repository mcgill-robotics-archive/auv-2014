/**
* Invoker loads task from xml
*/
#include "Invoker.h"

Invoker::Invoker(){
}

Invoker::~Invoker(){
	
}

void Invoker::AddTask(Task * t){
	TaskList.push_back(t);
}

void Invoker::StartRun(){
	int i;
	for(i = 0; i < TaskList.size(); i++){
		std::cout<<std::endl<<">>> Executing task: " << TaskList.at(i)->id <<std::endl;		
		TaskList.at(i)->Execute();
	}

}

//used for testing when you want to start at a certain task
void Invoker::StartAt(int n){
	//to make sure the program doesn't crash
	if(n >= TaskList.size() || n < 0) {
		std::cout << "INVALID STARTING TASK NUMBER, SKIPPING ALL TASKS" <<std::endl;
		return;
	}

	for(int i = n; i < TaskList.size(); i++){
		std::cout<<std::endl<<">>> Executing task: " << TaskList.at(i)->id <<std::endl;		
		TaskList.at(i)->Execute();
	}

}
