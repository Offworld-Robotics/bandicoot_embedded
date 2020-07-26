CC=gcc
CFLAGS=-Wall -Werror -Wextra -Wpedantic

all: pid fix_test

debug: all
debug: CFLAGS += -g -DDEBUG

obj/%.o: src/%.c
	$(CC) $(CFLAGS) -c -o $@ $< 

fix_test: obj/fix_t.o src/fix_test.c
	$(CC) $(CFLAGS) -o bin/fix_test $^

pid: obj/PIDController.o obj/fix_t.o src/pid_main.c
	$(CC) $(CFLAGS) -o bin/pid $^

clean:
	rm -rf *.o fix_test pid
