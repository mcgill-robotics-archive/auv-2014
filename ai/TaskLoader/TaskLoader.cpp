//Loader
	// takes string taskID (from xml file)  and returns corresponding task object
#include "TaskLoader.hpp"

void makeTask(std::string id){

	int t = TaskLoader::parseTask(id);
	int v = TaskLoader::parseVersion(id);

	switch(t){
		case 1:

			switch(v)
			{
				case 1:
					 //return new Task1v1();
					 break;
			break;

		case 2: 
			switch(v)
			{
				case 1:
					 //return new Task2v1();
					 break;
			}
			break;
	}
}

int main(){
	makeTask("1v1");
}