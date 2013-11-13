OBJS = Config.o
CC = g++
DEBUG = -g
CFLAGS = -Wall -Wextra -c
LFLAGS = -Wall $(DEBUG)

p1 : $(OBJS)
	$(CC) $(LFLAGS) $(OBJS) -o p1

Config.o : Config.h ../pugixml/src/pugixml.cpp Config.cpp 
	$(CC) $(CFLAGS) Config.cpp


