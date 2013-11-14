#ifndef Config_h
#define Config_h
#include <iostream>
#include <vector>


class Config {

	public:

	 	Config();

	    virtual void Loader();

		virtual std::vector<std::string> GetMap();

		virtual std::vector<std::string> GetTasks();   

	private:
	    std::string MapStartOfConfigFileName;
		std::string TasksStartOfConfigFileName;
	    std::string MapFile;
	    std::string TasksFile;
};

#endif // Config_h
