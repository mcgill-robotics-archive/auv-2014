#include "../pugixml/src/pugixml.cpp"
#include "Config.h"

using namespace std;

const char* taskFN = "tasks.xml"; //TODO change file name as neeeded
const char* mapFN = "map.xml";

pugi::xml_document loadFile(char* filename){
	pugi::xml_document roboSub_doc;
	pugi::xml_parse_result roboSub_result;
	//TODO check if the file exists
	
	roboSub_result = roboSub_doc.load_file(filename);
	if (roboSub_result){
   		std::cout << "XML [" << filename << "] loaded without errors\n\n";
	}
	else{
		std::cout << "XML [" << filename << "] loaded with errors\n";
		std::cout << "Error description: " << roboSub_result.description() << "\n";
		std::cout << "Error offset: " << roboSub_result.offset << " (error at [..." << (filename + roboSub_result.offset) << "]\n\n";
	}

	//deal with errors outside this to ensure program keeps running and errors are handeled accordingly
	return load_Success;
	
}


void parse(pugi::xml_document taskDoc, pugi::xml_document mapDoc){
	pugi::xml_node tasks, task_child, map, map_child;
	string task_list = "", map_list = "";


	tasks = taskDoc.child("TASK_LIST");
	map = mapDoc.child("MAP");	
	
	task_child = tasks.child("task");
	map_child = map.child("obj");
	//TODO In this area add any parsing needed for the TASK_LIST for example number of attributtes to a tag and maybe number of tasks?
	
	for(task_child; task_child; task_child = task_child.next_sibling("task")){
		task_list.append(task_child.next_attribute("name").value());
		//TODO add the other attributes here
		task_list.append("\n");
	}		
	
	for(map_child; map_child; map_child = map_child.next_sibling("obj")){
		std::cout << "2" << std::endl;
	}
}


Config::Config()
{

	pugi::xml_document roboSub_task_doc;
	pugi::xml_document roboSub_map_doc;
	
	roboSub_task_doc = 
	roboSub_map_doc = 


	//TODO create 
	//Constructor:
	int x = 0;
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

int main(){  //TODO Remove/COmment out later
	loadFile();
	parse();
	return 0;
}
