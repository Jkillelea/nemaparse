CC = gcc -Wall

all: main

main: main.o util.o constants.o

%.o: %.c
	$(CC) -c $< -o $@

.PHONY: clean
clean:
	rm -f *.o
