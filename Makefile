CC=gcc
CFLAGS=-c -Wall -Os
STRIP=strip
# g3_utils package, on debian provided by libsgutils2-dev
LIBS=-lsgutils2 -lpthread

all: syno_control

syno_control: syno_control.o
	$(CC) syno_control.o $(LIBS) -o syno_control
	$(STRIP) syno_control

syno_control.o: syno_control.c
	$(CC) $(CFLAGS) syno_control.c

clean:
	rm -rf *o syno_control