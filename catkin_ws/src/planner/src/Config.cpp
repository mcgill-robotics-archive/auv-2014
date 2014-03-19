/**
* @file Config.cpp
* @author Faraz Oman 
* @author Nick Margie
* @brief Config loads tasks from xml file, parses them, and then returns vector of strings with tasks.
*
*/
#include "Config.h"

using namespace std;


pugi::xml_document roboSub_task_doc;
pugi::xml_document roboSub_map_doc;
vector<string> taskList;
vector<string> mapList;


/**
* Constructor for Config object
*/


const char* taskFN = "tasks.xml"; //TODO change file name as neeeded


const char* mapFN = "map.xml";

/**
* Constructor for the Config object invokes the load file and parse methods (pugi) to parse the xml data needed.
* Values are accesible when requested from loader through getter methods 
*/

Config::Config()

{
	std::cout<<"Loading Task File"<< std::endl;

/**
* @param taskFN task ID and versionob
* @roboSub_task_doc llkj
*/
	loadFile(taskFN, roboSub_task_doc); //TODO deal with errors that arise if this returns false?
	//loadFile(mapFN, roboSub_map_doc); /*Used to load map if ever needed */
	parse(roboSub_task_doc, roboSub_map_doc);
	std::cout<<"Done Loading Task File"<< std::endl;
}

/**
* @param filename file name
* @param pugi::xml_document inheritance 
* 
* This takes as input a file name and reference to a         
* xml_document, used to load the file
*
*/
bool Config::loadFile(const char* filename, pugi::xml_document& roboSub_doc){
	pugi::xml_parse_result roboSub_result;
	bool is_valid = true;
	std::ofstream outfile;
	//TODO check if the file exists
	
	//used to check if file was loaded,includes other info
	roboSub_result = roboSub_doc.load_file(filename);

	//checks if the file was properly loaded and will add to the logger the result whether good or bad
	//TODO add time?
	if (roboSub_result){
		outfile.open("logger.txt", std::ios_base::app); 
   		outfile << "XML [" << filename << "] loaded without errors\n\n";
   		outfile.close();
	}
	else{
		is_valid = false;
		//TODO make a better logger message
		outfile.open("logger.txt", std::ios_base::app); 
		outfile << "XML [" << filename << "] loaded with errors\n";
		outfile << "Error description: " << roboSub_result.description() << "\n";
		outfile << "Error offset: " << roboSub_result.offset << " (error at [..." << (filename + roboSub_result.offset) << "]\n\n";
		outfile.close();
		std::cout << "Error loading the config files..."<< std::endl;
		std::cout<< "Make sure tasks.xml and map.xml are valid and located at the root of the catkin folder you are working in" <<std::endl;
		
	}

	//deal with errors outside this to ensure program keeps running and errors are handled accordingly
	return is_valid;
	
}


void Config::parse(pugi::xml_document& taskDoc, pugi::xml_document& mapDoc){
	pugi::xml_node tasks, task_child, map, map_child, map_gchild;
	string task_list = "";
	//string map_list = "";
	//below is used to know the size of the outer vector
	//int task_size = 0, map_size = 0;

	//xml_nodes to interate through and get the information
	tasks = taskDoc.child("TASK_LIST");
	//map = mapDoc.child("MAP");	

	//This is kept incase it is made into a 2D vector 
	//also it gets the size from the xml file
/*
	task_list = tasks.attribute("size").value();
	task_size = atoi(task_list.c_str());
	map_list = map.attribute("size").value();
	map_size = atoi(map_list.c_str());
*/
	//taskList.resize(task_size);
	//mapList.resize(map_size);

	task_child = tasks.child("task");
	//map_child = map.child("obj");
	//TODO In this area add any parsing needed for the TASK_LIST for example number of attributtes to a tag and maybe number of tasks?
	
	//iterates through the task nodes and adds it to the taskList vector object
	//add anymore attributes to be in the sml file to be parsed in this loop
	for(task_child; (task_child); task_child = task_child.next_sibling("task")){
		//taskList.push_back(task_child.attribute("name").value());
		taskList.push_back(task_child.attribute("id").value());
		//taskList.push_back(task_child.attribute("version").value());
		//taskList.push_back(task_child.attribute("type").value());
		//TODO add the other attributes here also try to make this more efficient/neater including the below for loop
	}		
	
	//this adds all the data from the xml file that contains the information about the map
	/*for(map_child; map_child; map_child = map_child.next_sibling("obj")){
		mapList.push_back(map_child.attribute("id").value());
		mapList.push_back(map_child.attribute("version").value());
		mapList.push_back(map_child.attribute("name").value());
		map_gchild = map_child.child("position");
		mapList.push_back(map_gchild.attribute("x").value());
		mapList.push_back(map_gchild.attribute("y").value());
		mapList.push_back(map_gchild.attribute("z").value());
		map_gchild = map_child.child("size");
		mapList.push_back(map_gchild.attribute("deltaX").value());
		mapList.push_back(map_gchild.attribute("deltaY").value());
		mapList.push_back(map_gchild.attribute("deltaZ").value());

	}*/

}


vector<string> Config::GetMap()
{
	//returns the maplist in a 1D vector object, is a vector of strings
	return mapList;
}

vector<string> Config::GetTasks()
{
	//returns the task list in a 1D string vector object
	return taskList;
}

//int main(){  //TODO Remove/COmment out later
//	Config test = Config();
//	return 0;
//}
