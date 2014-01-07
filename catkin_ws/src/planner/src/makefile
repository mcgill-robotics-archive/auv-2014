

Loader: TaskInvokerTester.o Loader.o TaskFactory.o Invoker.o quick_parse.o TaskConcrete.o Task.o Config.o pugixml.o
	clear
	g++ -o TaskInvokerTester TaskInvokerTester.o Loader.o TaskFactory.o Invoker.o quick_parse.o TaskConcrete.o Task.o Config.o pugixml.o
	make clean

TaskInvokerTester.o: TaskInvokerTester.cpp
	clear
	g++ -c TaskInvokerTester.cpp

Loader.o: Loader.cpp Loader.h
	clear
	g++ -c Loader.cpp

TaskFactory.o: TaskFactory.cpp TaskFactory.hpp
	clear
	g++ -c TaskFactory.cpp

Invoker.o: Invoker.cpp Invoker.h
	clear
	g++ -c Invoker.cpp

quick_parse.o: quick_parse.cpp quick_parse.hpp
	clear
	g++ -c quick_parse.cpp

TaskConcrete: TaskConcrete.o Task.o
	clear
	g++ -o TaskConcrete TaskConcrete.o Task.o

TaskConcrete.o: TaskConcrete.cpp TaskConcrete.hpp
	clear
	g++ -c TaskConcrete.cpp

Task.o: Task.cpp Task.hpp
	clear
	g++ -c Task.cpp

Config.o: Config.cpp Config.h
	clear
	g++ -c Config.cpp

pugixml.o: ../pugixml/src/pugixml.cpp ../pugixml/src/pugixml.hpp
	clear
	g++ -c ../pugixml/src/pugixml.cpp

clean:
	rm -vf *.o