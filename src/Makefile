#
# Makefile for mm.c
#
CC = gcc
CFLAGS = -Wall -O2 -m32

OBJS = mm.o memlib.o

mdriver: $(OBJS)
	$(CC) $(CFLAGS) -o mdriver $(OBJS)

memlib.o: memlib.c memlib.h
mm.o: mm.c mm.h memlib.h

clean:
	rm -f *~ *.o mdriver


