CC = g++
CFLAGS = -Wall -g
LDFLAGS = -lm


myprogram : Logger.o
	${CC} ${CFLAGS} Logger.o ${LDFLAGS} -o myProgram

logger.o : Logger.cpp
	${CC} ${CFLAGS} -c Logger.cpp 


clean : 
	rm -vf *.o

