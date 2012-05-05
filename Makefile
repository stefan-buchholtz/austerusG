GCC = clang -Wall

all: objdir austerus-send austerus-core

objdir:
	mkdir -p build

austerus-send: stats.o serial.o austerus-send.o
	$(GCC) -o austerus-send build/stats.o build/serial.o \
		build/austerus-send.o

austerus-core: serial.o austerus-core.o
	$(GCC) -o austerus-core build/serial.o build/austerus-core.o

austerus-send.o: src/austerus-send.c
	$(GCC) -c -o build/austerus-send.o src/austerus-send.c

austerus-core.o: src/austerus-core.c
	$(GCC) -c -o build/austerus-core.o src/austerus-core.c

serial.o: src/serial.c
	$(GCC) -c -o build/serial.o src/serial.c

stats.o: src/stats.c
	$(GCC) -c -o build/stats.o src/stats.c

clean:
	rm austerus-send austerus-core build/*

