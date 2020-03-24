all:	sim

sim:	SimApp.o PriorityQueue.h Node.h Queue.h Event.o EmptyDataCollectionException.o
	g++ -Wall -std=c++11 -o sim SimApp.o Event.o EmptyDataCollectionException.o

SimApp.o:	SimApp.cpp PriorityQueue.h Node.h Queue.h Event.h EmptyDataCollectionException.h
	g++ -c SimApp.cpp

Event.o:	Event.cpp Event.h
	g++ -c Event.cpp

EmptyDataCollectionException.o:	EmptyDataCollectionException.cpp EmptyDataCollectionException.h
	g++ -c EmptyDataCollectionException.cpp

clean:	
	rm -f sim *.o
