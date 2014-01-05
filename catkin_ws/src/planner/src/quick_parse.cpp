#include "quick_parse.hpp"

int parseTask(std::string id){
	unsigned int start = 0;
	unsigned int end;

	//finds the 'v' in the string and finds end index
	for(int i = 0; i < id.size(); i++){
		if(id.at(i) == 'v'){
			end = i;
		}
	}
	return atoi(id.substr(start,end).c_str());
}

int parseVersion(std::string id){
	unsigned int start;
	unsigned int end;

	for(int i = id.size()-1; i >= 0; i--){
		if(id.at(i) == 'v'){
			start = i + 1;
			end = id.size() - i;
		}
	}

	return atoi(id.substr(start,end).c_str());
	return 0;
}
/*
int main(){
	std::cout << parseTask("71v1");
	std::cout << parseVersion("71v1");
	return 0;
}
*/