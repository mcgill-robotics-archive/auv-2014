

TaskLoader: TaskLoader.o quick_parse.o
	clear
	g++ -o TaskLoader TaskLoader.o quick_parse.o

TaskLoader.o: TaskLoader.cpp TaskLoader.hpp
	clear
	g++ -c TaskLoader.cpp

quick_parse.o: quick_parse.cpp quick_parse.hpp
	clear
	g++ -c quick_parse.cpp

TaskConcrete: TaskConcrete.o Task.o
	clear
	g++ -o TaskConcrete TaskConcrete.o Task.o

TaskConcrete.o: TaskConcrete.cpp TaskConcrete.hpp
	clear
	g++ -c TaskConcrete.cpp

clean:
	rm -vf *.o