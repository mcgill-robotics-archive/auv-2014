//Loader

class TaskLoader{
	public:	
	// takes string taskID (from xml file)  and returns corresponding task object
		Task* makeTask(std::string id){

			int t = TaskLoader::parseTask(id);
			int v = TaskLoader::parseVersion(id);

			switch(t){
				case 1:

					switch(v)
					{
						case 1:
							 return new Task1v1();
							 break;
						case 2: 
							return new Task1v2();
							break;
						case 3: 
							return new Task1v3();
							break;
					}
					break;

				case 2: 
					switch(v)
					{
						case 1:
							 return new Task2v1();
							 break;
						case 2: 
							return new Task2v2();
							break;
						case 3: 
							return new Task2v2v3();
							break;
					}
					break;
			}
		}
}