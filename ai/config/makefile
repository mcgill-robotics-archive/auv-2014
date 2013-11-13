OBJS = Config.o
CC = g++
DEBUG = -g
CFLAGS = -Wall -c $(DEBUG)
LFLAGS = -Wall $(DEBUG)

p1 : $(OBJS)
	$(CC) $(LFLAGS) $(OBJS) -o p1

Config.o : Config.h Config.cpp 
	$(CC) $(CFLAGS) Config.cpp


