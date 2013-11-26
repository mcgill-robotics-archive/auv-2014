//TaskConcrete.cpp

class Task_1v1 : public Task
{
	public: 

		 std::string execute()
		 {
		 	return "1.1: Move Right, then fire Missiles!";
		 }
};

class Task_1v2 : public Task
{
	public: 

		 std::string execute()
		 {
		 	return "1.2: Move Left, then fire Missiles!";
		 }
};

class Task_1v3 : public Task
{
	public: 

		 std::string execute()
		 {
		 	return "1.3: Move Straight, then fire Missiles!";
		 }
};