PREFIX ?= /usr/local
INSTALL ?= install
BINDIR ?= $(PREFIX)/bin
MANDIR ?= $(PREFIX)/share/man
CC ?= gcc -Wall -pedantic -Wno-long-long -Wno-deprecated -ansi

UNAME := $(shell uname)

SETUID ?= 0

ifeq ($(SETUID),1)
COREFLAGS = -D SETUID
COREMODE = 6755
else
COREMODE = 0755
endif

all: objdir austerus-panel austerus-send austerus-verge austerus-core \
	austerus-shift

objdir:
	mkdir -p build

austerus-panel: nbgetline.o popen2.o serial.o austerus-panel.o
	$(CC) -o austerus-panel build/popen2.o build/nbgetline.o build/serial.o \
		build/austerus-panel.o -lncurses -lform

austerus-send: nbgetline.o popen2.o stats.o serial.o austerus-send.o
	$(CC) -o austerus-send build/nbgetline.o build/popen2.o build/stats.o \
		build/serial.o build/austerus-send.o -lm

austerus-verge: stats.o austerus-verge.o
	$(CC) -o austerus-verge build/stats.o build/austerus-verge.o -lm

ifeq ($(UNAME), Darwin)
austerus-shift: stats.o austerus-shift.o fmemopen.o
	$(CC) -o austerus-shift build/stats.o build/fmemopen.o build/austerus-shift.o -lm
else
austerus-shift: stats.o austerus-shift.o
	$(CC) -o austerus-shift build/stats.o build/austerus-shift.o -lm
endif

austerus-core: serial.o austerus-core.o
	$(CC) -o austerus-core build/serial.o build/austerus-core.o

austerus-panel.o: src/austerus-panel.c
	$(CC) -c -o build/austerus-panel.o src/austerus-panel.c

austerus-send.o: src/austerus-send.c
	$(CC) -c -o build/austerus-send.o src/austerus-send.c

austerus-verge.o: src/austerus-verge.c
	$(CC) -c -o build/austerus-verge.o src/austerus-verge.c

austerus-shift.o: src/austerus-shift.c
	$(CC) -c -o build/austerus-shift.o src/austerus-shift.c

austerus-core.o: src/austerus-core.c
	$(CC) $(COREFLAGS) -c -o build/austerus-core.o src/austerus-core.c

serial.o: src/serial.c
	$(CC) -c -o build/serial.o src/serial.c

stats.o: src/stats.c
	$(CC) -c -o build/stats.o src/stats.c

popen2.o: src/popen2.c
	$(CC) -c -o build/popen2.o src/popen2.c

nbgetline.o: src/nbgetline.c
	$(CC) -c -o build/nbgetline.o src/nbgetline.c

ifeq ($(UNAME), Darwin)
fmemopen.o:
	$(CC) -c -o build/fmemopen.o src/fmemopen/fmemopen.c
endif

install:
	$(INSTALL) -d $(DESTDIR)$(BINDIR)
	$(INSTALL) -d $(DESTDIR)$(MANDIR)/man1
	$(INSTALL) -m $(COREMODE) austerus-core $(DESTDIR)$(BINDIR)
	$(INSTALL) -m 0755 austerus-send $(DESTDIR)$(BINDIR)
	$(INSTALL) -m 0755 austerus-panel $(DESTDIR)$(BINDIR)
	$(INSTALL) -m 0755 austerus-verge $(DESTDIR)$(BINDIR)
	$(INSTALL) -m 0755 austerus-shift $(DESTDIR)$(BINDIR)
	$(INSTALL) -m 0644 docs/austerus-core.1 $(DESTDIR)$(MANDIR)/man1
	$(INSTALL) -m 0644 docs/austerus-verge.1 $(DESTDIR)$(MANDIR)/man1

clean:
	rm austerus-panel austerus-send austerus-core austerus-verge \
		austerus-shift build/*

