#ifndef Config_h
#define Config_h
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

using namespace std;

class Config {

 public:

    virtual void Loader();

    virtual void GetMap();

    virtual void GetTasks();

    virtual void newOperation();

   


 private:
    string MapStartOfConfigFileName;
	string TasksStartOfConfigFileName;
    string MapFile;
    string TasksFile;
};

#endif // Config_h
