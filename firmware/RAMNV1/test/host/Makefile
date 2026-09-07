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
            -DTELEMATICS_HAS_STREAM_STATE \
            -Istubs -I. -I$(CORE)/Inc -I$(CORE)/Src
BIN      := run_tests
SRCS     := test_telematics_spi.c test_conformance.c fakes.c harness.c

.PHONY: all clean
all: $(BIN)
	./$(BIN)

$(BIN): $(SRCS) $(wildcard stubs/*.h) $(CORE)/Src/ramn_telematics.c
	$(CC) $(CFLAGS) -o $@ $(SRCS)

clean:
	rm -f $(BIN)

# Run this same suite against an older revision of the module under test, to
# check whether a failure is a regression or was always broken:
#
#   make control REV=776ee66
#
# A control run that reports "markers to remove" means the CHECK_BUG defects
# do not exist at REV -- i.e. they were introduced after it.
CONTROL_DIR := .control
control:
	@test -n "$(REV)" || { echo "usage: make control REV=<git-rev>"; exit 2; }
	@mkdir -p $(CONTROL_DIR)
	git show $(REV):firmware/RAMNV1/Core/Src/ramn_telematics.c > $(CONTROL_DIR)/ramn_telematics.c
	@echo "--- suite vs $(REV) ---"
	$(CC) -std=c11 -g -O0 -w -I$(CONTROL_DIR) -Istubs -I. -I$(CORE)/Inc \
	      -o $(CONTROL_DIR)/run_control $(SRCS)
	-./$(CONTROL_DIR)/run_control

.PHONY: control
