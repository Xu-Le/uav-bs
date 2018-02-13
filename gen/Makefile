# This file is part of IFDBSP
CC = g++
CXXFLAGS = -Wall -g -std=c++11
genCases: Perlin.o genCases.o
	$(CC) -o $@ $^
Perlin.o: Perlin.cpp Perlin.h
	$(CC) $(CXXFLAGS) -c $<
genCases.o: genCases.cpp
	$(CC) $(CXXFLAGS) -c $<
.PHONY: clean
clean:
	rm -f *.h~
	rm -f *.cpp~
	rm -f *.o
	rm -f genCases
