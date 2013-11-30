#include "Loader.h"
#include <vector>

int main(){

	std::cout<<"The Task Test!" << std::endl;
	Loader* ldr = new Loader();
	ldr->reLoadInvoker();
	Invoker* inv = ldr->getInvoker();

	inv->StartRun();

	return 0;
}