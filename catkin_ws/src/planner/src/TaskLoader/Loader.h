#ifndef Loader_h
#define Loader_h

#include "Invoker.h"
#include "Config.h"
#include "TaskFactory.hpp"

class Loader {

 public:

 	void loadInvoker();

 	void reLoadInvoker();

 	Invoker* getInvoker();

    Loader();

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
