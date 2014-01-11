//Loader
	// takes string taskID (from xml file)  and returns corresponding task object
#include "TaskFactory.hpp"

TaskFactory::TaskFactory(Interface* inf){
	main_interface = inf;

}

Task* TaskFactory::makeTask(std::string id){

	//parses the string into the id task number and id version number 
	int t = parseTask(id);
	int v = parseVersion(id);

	//switch case statement to determine which task object needs to be created
	//Everytime a task is added add the swtich case for it accordingly
	//the first case is for the task number and then inside that case another swtich
	//to determie the version number
	switch(t){
		case 1:

			switch(v){
				case 1:
					 return new Task_1v1(main_interface);//return new Task1v1();
					 break;

				case 2:
					return new Task_1v2(main_interface);

				case 3: 
					return new TorpedoTask(main_interface);

				default:
					return new Task(main_interface);
					break;
			}
			break;


		case 2: 
			switch(v){
			
				case 1:
					return new Task_2v1(main_interface);
					 //return new Task2v1();
					 break;
			
				default:
					return new Task(main_interface);
					break;
			}
			break;

		default:
			return new Task(main_interface);
			break;
	}
	return new Task();
}


//int main(){
//	Task* MyTask = makeTask("1v1");
//	MyTask->execute();
//	std::cout << MyTask->id;
//	return 0;
//}	