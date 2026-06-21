CC = gcc
CFLAGS = -Wall -Wextra -std=c99 -I../../Core/Inc
SRC = test_main.c test_cj702.c test_aggregator.c test_lora_frame.c \
      ../../Core/Src/cj702.c ../../Core/Src/aggregator.c ../../Core/Src/lora_frame.c

.PHONY: all clean

all: host_tests
	./host_tests

host_tests: $(SRC)
	$(CC) $(CFLAGS) -o $@ $(SRC)

clean:
	rm -f host_tests
