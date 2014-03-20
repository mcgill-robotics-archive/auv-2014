//Loader
	// takes string taskID (from xml file)  and returns corresponding task object
#include "TaskFactory.hpp"

TaskFactory::TaskFactory(){

}

//Parses the task number from the tasks.xml file 
int TaskFactory::parseTask(std::string id){
	unsigned int start = 0;
	unsigned int end;

	//finds the 'v' in the string and finds end index
	for(int i = 0; i < id.size(); i++){
		if(id.at(i) == 'v'){
			end = i;
		}
	}
	return atoi(id.substr(start,end).c_str());
}

/**
*Parses the version section in that comes from the tasks.xml file
*/
int TaskFactory::parseVersion(std::string id){
	unsigned int start;
	unsigned int end;

	for(int i = id.size()-1; i >= 0; i--){
		if(id.at(i) == 'v'){
			start = i + 1;
			end = id.size() - i;
		}
	}

	return atoi(id.substr(start,end).c_str());

}

Task* TaskFactory::getEndTask(){
	return new Task_Kill();	
}

Task* TaskFactory::getGateTask(){
	return new Task_Gate();	
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
					 return new Task_Gate();//return new Task1v1();
					 break;

				case 2:
					return new Task_1v2();
					break;
				case 3: 
					return new TorpedoTask();
					break;
				default:
					return new Task();
					break;
			}
			break;


		case 2: 
			switch(v){
			
				case 1:
					return new Task_Lane();
					break;
			
				default:
					return new Task();
					break;
			}
			break;

		default:
			return new Task();
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
