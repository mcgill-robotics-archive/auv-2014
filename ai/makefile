CC = g++
CFLAGS = -Wall -g
LDFLAGS = -lm


myprogram : Logger.o
	${CC} ${CFLAGS} Logger.o ${LDFLAGS} -o myProgram

logger.o : Logger.cpp Logger.h
	${CC} ${CFLAGS} -c Logger.cpp Logger.h


clean : 
	rm -vf *.o

