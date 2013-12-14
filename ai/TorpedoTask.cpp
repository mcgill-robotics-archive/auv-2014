	#include "Task.h"


class TorpedoTask: public Task{
public:
	int Execute(){

		std:cout << "Shoot Torpedo!!!"<< std::endl;
		return 0;
	}
};