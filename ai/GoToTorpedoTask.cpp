#include "Task.h"


class GoToTorpedoTask: public Task{
public:
	int Execute(){

		std:cout << "Going to a torpedo task." << std::endl;
		return 0;
	}
};
