CC ?= cc
CFLAGS ?= -Wall -Wextra -O2

all: tf2agw

tf2agw: tf2agw.c
	$(CC) $(CFLAGS) -o $@ $< -lutil

clean:
	rm -f tf2agw
