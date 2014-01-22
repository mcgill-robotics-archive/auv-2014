//Task.cpp
//Task Interface (abstract)
#include "Task.hpp" 
#include "Interface.h"

Task::Task(){
	id = "default_id";
}

Task::Task(std::string myid){
	id = myid;
}

int Task::Execute() {
	std::cout << id;
};
