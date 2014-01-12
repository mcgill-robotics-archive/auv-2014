#include "Task.h"


class Task_Buoy: public Task{
public:
	int Execute(){

		std:cout << "Executing Buoy Task" << std::endl;
		return 0;
	}
};
