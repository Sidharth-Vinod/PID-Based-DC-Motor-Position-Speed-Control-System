CC := gcc
CFLAGS := -std=c99 -D_DEFAULT_SOURCE -Wall -Wextra -O2 -I../../include
LDLIBS := -lm

SRCS := sim_main.c motor_model.c ../../src/pid.c ../../src/protocol.c
BIN := motor_sim

.PHONY: all run clean
all: $(BIN)

$(BIN): $(SRCS)
	$(CC) $(CFLAGS) -o $(BIN) $(SRCS) $(LDLIBS)

run: $(BIN)
	./$(BIN)

clean:
	rm -f $(BIN)
