//Task.cpp
//Task Interface (abstract)
#include "Task.hpp" 

Task::Task(Interface* inf){
	cur_interface = inf;
	id = "default_id";
}

Task::Task(std::string myid, Interface* interface){
	id = myid;
	cur_interface = interface;
}

int Task::Execute() {
	std::cout << id;
};
