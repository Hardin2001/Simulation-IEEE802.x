all: main

main: main.o
	g++ main.o -o main.out
main.o: main.cpp
	g++ -c main.cpp
clean:
	rm main.o main.out
	