# Host tests for RAMN firmware logic. Plain gcc, no ARM toolchain, no board.
#
#   make          build and run
#   make clean
#
# Sources under test are compiled for the host against the stubs in stubs/.
# ramn_config.h is the REAL one from Core/Inc so CAN IDs cannot drift.

CORE     := ../../Core
CFLAGS   := -std=c11 -g -O0 -Wall -Wextra \
            -Wno-unused-parameter -Wno-unused-function -Wno-unused-variable \
            -Istubs -I. -I$(CORE)/Inc -I$(CORE)/Src
BIN      := run_tests
SRCS     := test_telematics_spi.c fakes.c harness.c

.PHONY: all clean
all: $(BIN)
	./$(BIN)

$(BIN): $(SRCS) $(wildcard stubs/*.h) $(CORE)/Src/ramn_telematics.c
	$(CC) $(CFLAGS) -o $@ $(SRCS)

clean:
	rm -f $(BIN)
