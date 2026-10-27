CC ?= gcc
CFLAGS ?= -O2 -Wall -Wextra -std=c11
SRC_DIR := ../../src

SOURCES := trickle_bench.c $(SRC_DIR)/trickle_model.c $(SRC_DIR)/powder_sim.c
HEADERS := $(SRC_DIR)/trickle_model.h $(SRC_DIR)/powder_sim.h

all: trickle_bench trickle_test trickle_export

trickle_bench: $(SOURCES) $(HEADERS)
	$(CC) $(CFLAGS) -I$(SRC_DIR) -o $@ $(SOURCES) -lm

trickle_test: trickle_test.c $(SRC_DIR)/trickle_model.c $(SRC_DIR)/powder_sim.c $(SRC_DIR)/sha256.c $(HEADERS)
	$(CC) $(CFLAGS) -I$(SRC_DIR) -o $@ trickle_test.c $(SRC_DIR)/trickle_model.c $(SRC_DIR)/powder_sim.c $(SRC_DIR)/sha256.c -lm

trickle_export: trickle_export.c $(SRC_DIR)/trickle_model.c $(SRC_DIR)/powder_sim.c $(HEADERS)
	$(CC) $(CFLAGS) -I$(SRC_DIR) -o $@ trickle_export.c $(SRC_DIR)/trickle_model.c $(SRC_DIR)/powder_sim.c -lm

run: all
	./trickle_test
	./trickle_bench

clean:
	rm -f trickle_bench trickle_test trickle_export

.PHONY: all run clean
