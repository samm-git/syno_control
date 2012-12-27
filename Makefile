CC=gcc
CFLAGS=-c -Wall
STRIP=strip
# g3_utils package, on debian provided by libsgutils2-dev
LIBS=-lsgutils2

all: status_led

status_led: status_led.o
	$(CC) status_led.o $(LIBS) -o status_led
	$(STRIP) status_led

status_led.o: status_led.c
	$(CC) $(CFLAGS) status_led.c

clean:
	rm -rf *o status_led