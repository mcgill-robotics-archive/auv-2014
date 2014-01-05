#include "Task.h"
#include "TorpedoTask.cpp"
#include "GoToTorpedoTask.cpp"
#include <vector>
#include "Invoker.cpp"

int main(){

	std::cout<<"The Task Test!" << std::endl;
	TorpedoTask * task = new TorpedoTask();
	GoToTorpedoTask * go = new GoToTorpedoTask();

	std::vector<Task*> TaskList;
	TaskList.push_back(go);
	TaskList.push_back(task);

	//TaskList[0]->Execute();
	//TaskList[1]->Execute();

	Invoker inv;
	inv.AddTask(go);
	inv.AddTask(task);
	inv.AddTask(go);
	inv.AddTask(task);
	inv.StartRun();

	cout<< "Size: " << TaskList.size() << endl;
	return 0;
}