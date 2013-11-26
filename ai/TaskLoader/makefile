

TaskLoader: TaskLoader.o quick_parse.o
	clear
	g++ -o TaskLoader TaskLoader.o quick_parse.o

TaskLoader.o: TaskLoader.cpp TaskLoader.hpp
	clear
	g++ -c TaskLoader.cpp

quick_parse.o: quick_parse.cpp quick_parse.hpp
	clear
	g++ -c quick_parse.cpp

clean:
	rm -vf *.o