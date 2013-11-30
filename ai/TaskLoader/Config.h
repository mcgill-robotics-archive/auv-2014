#ifndef Config_h
#define Config_h
#include <iostream>
#include <vector>
#include <fstream>
#include "../pugixml/src/pugixml.hpp"

using namespace std;

class Config{

	public:

		Config();

	    void Loader();

		vector<string> GetMap();

		vector<string> GetTasks();

		bool loadFile(const char* filename, pugi::xml_document& roboSub_doc);

		void parse(pugi::xml_document& taskDoc, pugi::xml_document& mapDoc);

	private:
	    std::vector<string> taskList;
	    std::vector<string> mapList;
	    std::string MapFile;
	    std::string TasksFile;
};

#endif // Config_h
