//TaskConcrete.cpp

#include "TaskConcrete.hpp"


		 void Task_1v1::execute()
		 {
		 	std::cout<< "1.1: Move Right, then fire Missiles!" << std::endl;
		 }

		 void Task_1v2::execute()
		 {
		 	std::cout<< "1.2: Move Left, then fire Missiles!" << std::endl;
		 }

		 void Task_1v3::execute()
		 {
		 	std::cout<< "1.3: Move Straight, then fire Missiles!" << std::endl;
		 }


/*int main(){
	Task* MyTask = new Task_1v3();
	MyTask->execute();
return 0;
}*/