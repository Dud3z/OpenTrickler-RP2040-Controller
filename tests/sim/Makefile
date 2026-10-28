CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -std=c11
SRC_DIR := ../../src

SOURCES := trickle_bench.c $(SRC_DIR)/trickle_model.c $(SRC_DIR)/powder_sim.c
HEADERS := $(SRC_DIR)/trickle_model.h $(SRC_DIR)/powder_sim.h

all: trickle_bench trickle_test trickle_export guard_test

trickle_bench: $(SOURCES) $(HEADERS)
	$(CC) $(CFLAGS) -I$(SRC_DIR) -o $@ $(SOURCES) -lm

trickle_test: trickle_test.c $(SRC_DIR)/trickle_model.c $(SRC_DIR)/powder_sim.c $(SRC_DIR)/sha256.c $(HEADERS)
	$(CC) $(CFLAGS) -I$(SRC_DIR) -o $@ trickle_test.c $(SRC_DIR)/trickle_model.c $(SRC_DIR)/powder_sim.c $(SRC_DIR)/sha256.c -lm

trickle_export: trickle_export.c $(SRC_DIR)/trickle_model.c $(SRC_DIR)/powder_sim.c $(HEADERS)
	$(CC) $(CFLAGS) -I$(SRC_DIR) -o $@ trickle_export.c $(SRC_DIR)/trickle_model.c $(SRC_DIR)/powder_sim.c -lm

guard_test: guard_test.c $(SRC_DIR)/charge_guard.c $(SRC_DIR)/charge_guard.h
	$(CC) $(CFLAGS) -I$(SRC_DIR) -o $@ guard_test.c $(SRC_DIR)/charge_guard.c -lm

run: all
	./guard_test
	./trickle_test
	./trickle_bench

clean:
	rm -f trickle_bench trickle_test trickle_export guard_test

.PHONY: all run clean
