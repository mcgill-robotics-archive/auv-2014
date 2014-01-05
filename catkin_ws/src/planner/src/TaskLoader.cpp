//Loader
	// takes string taskID (from xml file)  and returns corresponding task object
#include "TaskFactory.hpp"

Task* makeTask(std::string id){

	int t = parseTask(id);
	int v = parseVersion(id);

	switch(t){
		case 1:

			switch(v){
				case 1:
					 return new Task_1v1();//return new Task1v1();
					 break;
			}
			break;


		case 2: 
			switch(v){
				case 1:
					return new Task_1v2();
					 //return new Task2v1();
					 break;
			}
			break;
		
	}
	return NULL;
}


int main(){
	Task* MyTask = makeTask("1v1");
	MyTask->execute();
	std::cout << MyTask->id;
	return 0;
}	