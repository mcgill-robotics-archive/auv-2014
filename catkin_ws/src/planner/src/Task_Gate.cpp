#include "Task.h"


class Task_Gate: public Task{
public:
	int Execute(){

		std:cout << "Executing Gate Task" << std::endl;
		return 0;
	}
};
