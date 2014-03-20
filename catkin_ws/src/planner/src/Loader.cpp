/**
* @file Loader.cpp
* @version 1.0
*
*/


#include "Loader.h"

void ps3Control(); 

Loader::Loader(){
/**
* Initialize the objects
*/
	std::cout<<"In Loader Constructor"<< std::endl;
	main_Invoker = new Invoker();
	main_Config = new Config();
	main_Taskfactory = new TaskFactory();

/**
* loads the invoker
*/ 
	isInvokerLoaded = false;

	//int i = 0;
	//while(i < 1000000){
	//	ps3Control(); i++;
	//}
	loadInvoker();


}

/**
* loadInvoker() goes into config and gets the task list, then pulls the tasks from taskFactory, and feeds task objects to invoker
*/
void Loader::loadInvoker(){
/**
* will only run if Invoker is not loaded yet
*/
	if(!isInvokerLoaded){
		std::vector<std::string> s_tasklist;

/**
* gets the vector of tasks fron the config
*/
		s_tasklist = main_Config->GetTasks();

/**
* uses the vector of strings to input into the task factory to * then add a task to the invoker
*/
		for(int i = 0; i < s_tasklist.size(); i++){
			std::cout<<"Adding task to invoker"<< std::endl;
			main_Invoker->AddTask(main_Taskfactory->makeTask(s_tasklist.at(i)));
		}
		//std::cout<<"Adding default gate task because config does not work..."<< std::endl; // works now!!!
		//main_Invoker->AddTask(main_Taskfactory->makeTask("1v1")); // works now!!!
	}

main_Invoker->AddTask(main_Taskfactory->getGateTask());
	//adds the end task after all other tasks have been loaded
	main_Invoker->AddTask(main_Taskfactory->getEndTask());

	//To make sure the invoker doesn't get loaded agian unless needed
	isInvokerLoaded = true;
	std::cout<<"Done loading invoker"<< std::endl;
}

/**
* incase the Invoker wants to be reloaded
*/
void Loader::reLoadInvoker(){
/**
* change variable to false for the loadInvoker() call
*/	
	isInvokerLoaded = false;
	main_Invoker = new Invoker();
	loadInvoker();
}

Invoker* Loader::getInvoker(){
	return main_Invoker;
}


//probably not used?
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
