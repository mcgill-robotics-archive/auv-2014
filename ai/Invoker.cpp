#include "Invoker.h"

void Invoker::AddTask(Task * t){
	TaskList.push_back(t);
}

void Invoker::StartRun(){
	int i;
	for(i = 0; i < TaskList.size(); i++){
		TaskList[i]->Execute();
	}

}