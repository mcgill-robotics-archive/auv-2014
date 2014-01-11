#ifndef Loader_h
#define Loader_h

#include "Invoker.h"
#include "Config.h"
#include "TaskFactory.hpp"
#include "Interface.h"

class Loader {

 public:

 	void loadInvoker();

 	void reLoadInvoker();

 	Invoker* getInvoker();

    Loader(Interface* inf);

    void GetMap();

    void GetTasks();

    void newOperation();

 private:
 	Interface* main_Interface;
    Invoker* main_Invoker;
    Config* main_Config;
    TaskFactory* main_Taskfactory;

    bool isInvokerLoaded;
};

#endif // Loader_h
