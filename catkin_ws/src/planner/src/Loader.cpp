#include "Loader.h"

void ps3Control(); 

Loader::Loader(){
	//Initialize the objects
	main_Invoker = new Invoker();
	main_Config = new Config();
	main_Taskfactory = new TaskFactory();
	//loads the invoker
	isInvokerLoaded = false;
int i = 0;
while(i < 1000000){
	ps3Control(); i++;}
	loadInvoker();
}

//This loads the Invoker by adding the tasks to it as needed
void Loader::loadInvoker(){
	//will only run if Invoker is not loaded yet
	if(!isInvokerLoaded){
		std::vector<std::string> s_tasklist;

		//gets the vector of tasks fron the config
		s_tasklist = main_Config->GetTasks();

		//uses the vector of strings to input into the task factory to then add a task to the invoker
		for(int i = 0; i < s_tasklist.size(); i++){
			main_Invoker->AddTask(main_Taskfactory->makeTask(s_tasklist.at(i)));
		}
	}
	//To make sure the invoker doesn't get loaded agian unless needed
	isInvokerLoaded = true;
}

//incase the Invoker wants to be reloaded
void Loader::reLoadInvoker(){
	//chage variable to false for the loadInvoker() call
	isInvokerLoaded = false;
	main_Invoker = new Invoker();
	loadInvoker();
}

Invoker* Loader::getInvoker(){
	return main_Invoker;
}

void Loader::GetMap()
{
	//Figure out a way to return all the info in the map.
	//Maybe return vector of pose objects where Id is the 
	//name of the task
}

void Loader::GetTasks(){




}

void Loader::newOperation(){

}
