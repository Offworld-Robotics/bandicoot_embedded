CC=gcc
CFLAGS=-Wall -Werror -Wextra -Wpedantic

all: pid fix_test

debug: all
debug: CFLAGS += -g

%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $< 

fix_test: fix_t.o fix_test.c
	$(CC) $(CFLAGS) -o fix_test fix_t.o fix_test.c

pid: PIDController.o fix_t.o pid_main.c
	$(CC) $(CFLAGS) -o pid fix_t.o PIDController.o pid_main.c

clean:
	rm -rf *.o fix_test pid
