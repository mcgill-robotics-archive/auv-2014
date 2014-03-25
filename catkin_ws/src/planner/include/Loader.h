/**
* @file Loader.h
* @version 1.0.0 
* @brief loader.h initializes some functions
*/

#ifndef Loader_h
#define Loader_h

#include "Invoker.h"
#include "Config.h"
#include "TaskFactory.hpp"
#include "Interface.h"
#include "Task.hpp"
class TaskFactory;
class Loader {

 public:

    Loader(std::string xmlFilesPath);

    //~Loader();

    void loadInvoker();

    void reLoadInvoker();

    Invoker* getInvoker();

    void GetMap();

    void GetTasks();

    void newOperation();

 private:
 	
    Invoker* main_Invoker;
    Config* main_Config;
    TaskFactory* main_Taskfactory;

    bool isInvokerLoaded;
};

#endif // Loader_h
