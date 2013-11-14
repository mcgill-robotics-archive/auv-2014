#ifndef Config_h
#define Config_h
#include <iostream>
#include <vector>

using namespace std;

class Config {

	public:

		Config();

	    void Loader();

		void GetMap();

		void GetTasks();   

	private:
	    std::string MapStartOfConfigFileName;
		std::string TasksStartOfConfigFileName;
	    std::string MapFile;
	    std::string TasksFile;
};

#endif // Config_h
