#ifndef Config_h
#define Config_h
#include <iostream>
#include <vector>
#include <fstream>

using namespace std;

class Config {

	public:

		Config();

	    void Loader();

		vector<string> GetMap();

		vector<string> GetTasks();   

	private:
	    std::vector<string> taskList;
	    std::vector<string> mapList;
	    std::string MapFile;
	    std::string TasksFile;
};

#endif // Config_h
