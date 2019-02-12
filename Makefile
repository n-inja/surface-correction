# Makefile
main: main.cpp
	g++ -Wall -O2 -std=c++14 -o main `pkg-config opencv4 --cflags` `pkg-config opencv4 --libs` main.cpp

viewer: viewer.cpp
	g++ -Wall -O2 -std=c++14 -o viewer `pkg-config opencv4 --cflags` `pkg-config opencv4 --libs` viewer.cpp
