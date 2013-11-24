OBJS = Config.o
CC = g++
DEBUG = -g
CFLAGS = -Wall
LFLAGS = -Wall $(DEBUG)

Config.out : Config.o pugixml.o
	$(CC) $(CFLAGS) -o Config.out Config.o pugixml.o

Config.o : Config.cpp Config.h
	$(CC) $(CFLAGS) -c Config.cpp

pugixml.o : ../pugixml/src/pugixml.cpp ../pugixml/src/pugixml.hpp
	$(CC) $(CFLAGS) -c ../pugixml/src/pugixml.cpp

clean :
	rm -vf *.o


