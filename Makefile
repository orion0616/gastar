CC= nvcc
OP= -O3 -std=c++11

all: main.o Entry.o Timer.o ScenarioLoader.o
	$(CC) $(OP) main.o Entry.o Timer.o ScenarioLoader.o
main.o: main.cpp
	$(CC) $(OP) -c main.cpp
Entry.o: Entry.cu Entry.h BinaryHeap.h
	$(CC) $(OP) -c Entry.cu
Timer.o: Timer.cpp
	$(CC) $(OP) -c Timer.cpp
Scenario.o: ScenarioLoader.cpp
	$(CC) $(OP) -c ScenarioLoader.cpp
clean:
	rm a.out *.o
