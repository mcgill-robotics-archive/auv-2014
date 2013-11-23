OBJS = Config.o
CC = g++
DEBUG = -g
CFLAGS = -Wall -Wextra -c
LFLAGS = -Wall $(DEBUG)

p1 : $(OBJS)
	$(CC) $(LFLAGS) $(OBJS) -o p1

Config.o : Config.cpp Config.h ../pugixml/src/pugixml.h
	$(CC) $(CFLAGS) -c Config.cpp Config.h ../pugixml/src/pugixml.h

clean :
	rm -vf *.o


