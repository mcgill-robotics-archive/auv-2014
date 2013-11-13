#include "../pugixml/src/pugixml.cpp"
#include "Config.h"


pugi::xml_document roboSub_doc;
pugi::xml_parse_result roboSub_result;
const char* filename = "tree.xml";

bool loadFile(){
	bool load_Success = true;

	//TODO check if the file exists
	
	roboSub_result = roboSub_doc.load_file(filename);
	if (roboSub_result){
   		std::cout << "XML [" << filename << "] parsed without errors\n\n";
	}
	else{
		std::cout << "XML [" << filename << "] parsed with errors\n";
		std::cout << "Error description: " << roboSub_result.description() << "\n";
		std::cout << "Error offset: " << roboSub_result.offset << " (error at [..." << (filename + roboSub_result.offset) << "]\n\n";

		load_Success = false;
	}

	//deal with errors outside this to ensure program keeps running and errors are handeled accordingly
	return load_Success;
	
}


void parse(){
	pugi::xml_node tasks, task_child, map, map_child;

	tasks = roboSub_doc.child("TASK_LIST");
	map = roboSub_doc.child("MAP");

	
	
	task_child = tasks.child("task");
	map_child = map.child("Obj");
	//TODO In this area add any parsing needed for the TASK_LIST for example number of attributtes to a tag and maybe number of tasks?
	
	std::cout << task_child.value();
	for(task_child; task_child; task_child = task_child.next_sibling("task")){
		std::cout << "1" << std::endl;
	}		
	
	for(map_child; map_child; map_child = map_child.next_sibling("obj")){
		std::cout << "2" << std::endl;
	}
}


void Config::Config()
{
	//TODO create 
	//Constructor:
	//Open file with name = MapStartOConfigFileName * .txt
	//(so that starts with MapStartOConfigFileName, may have 
	//lots of other descriptive stuff after that and then has .txt
	//Do the same for TasksFile
	//** must work on both Ubunto and Windows
}

void Config::GetMap()
{
	//Figure out a way to return all the info in the map.
	//Maybe return vector of pose objects where Id is the 
	//name of the task
}

void Config::GetTasks()
{
	//Figure out a way to return all the info for the task list.
	//Maybe return vector of strings with the Id of the task
}

void Config::newOperation() //TODO Remove
{
}

int main(){  //TODO Remove/COmment out later
	loadFile();
	parse();
	return 0;
}
