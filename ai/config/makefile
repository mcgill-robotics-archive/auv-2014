OBJS = Config.o
CC = g++
DEBUG = -g
CFLAGS = -Wall -Wextra -c
LFLAGS = -Wall $(DEBUG)

p1 : $(OBJS)
	$(CC) $(LFLAGS) $(OBJS) -o p1

Config.o : Config.cpp Config.h ../pugixml/src/pugixml.cpp
	$(CC) $(CFLAGS) -c Config.cpp Config.h pugixml.cpp

clean :
	rm -vf *.o


