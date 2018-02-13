# This file is part of IFDBSP
CC = g++
CXXFLAGS = -Wall -g -std=c++11
UAV: Log.o Utils.o UAV.o IFDBSP.o main.o
	$(CC) -o $@ $^
Log.o: Log.cpp Log.h
	$(CC) $(CXXFLAGS) -c $<
Utils.o: Utils.cpp Utils.h
	$(CC) $(CXXFLAGS) -c $<
UAV.o: UAV.cpp UAV.h
	$(CC) $(CXXFLAGS) -c $<
IFDBSP.o: IFDBSP.cpp IFDBSP.h UAV.h Timer.h
	$(CC) $(CXXFLAGS) -c $<
main.o: main.cpp IFDBSP.h UAV.h
	$(CC) $(CXXFLAGS) -c $<
.PHONY: clean
clean:
	rm -f *.csv
	rm -f *.h~
	rm -f *.cpp~
	rm -f *.o
	rm -f UAV
