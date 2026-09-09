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
SECOC    := $(CORE)/Src/ramn_secoc.c $(CORE)/Src/ramn_blake2s.c \
            $(CORE)/Src/ramn_secoc_keys.c $(CORE)/Src/ramn_secoc_session.c \
            $(CORE)/Src/ramn_secoc_link.c
SRCS     := test_telematics_spi.c test_conformance.c fakes.c harness.c $(SECOC)

# ECU A's image receive path is a separate binary: it compiles a different
# module white-box, and two files cannot each #include a .c into one link.
BIN_A    := run_tests_ecua
SRCS_A   := test_screen_image.c fakes_screen.c fakes.c harness.c $(SECOC)
# ENABLE_SCREEN comes from the real ramn_config.h for an ECU A build, so it is
# not defined here -- doing so shadows the project's own definition.
CFLAGS_A := $(CFLAGS) -Wno-unused-but-set-variable

.PHONY: all clean ecud ecua targets
all: ecud ecua targets

# Catches what a host suite structurally cannot: a module that compiles and
# passes every test here, then fails to LINK for a target that lacks the
# symbols it references. See check_targets.sh.
targets:
	./check_targets.sh

ecud: $(BIN)
	./$(BIN)

ecua: $(BIN_A)
	./$(BIN_A)

# This suite exercises ECU D's half of the link, but ramn_config.h's default
# target is ECU A, so the SENDER role is not selected by the config here. Ask
# for it explicitly rather than switching the whole build to TARGET_ECUD, which
# would also pull in ENABLE_UART and change what these tests compile.
$(BIN): $(SRCS) $(wildcard stubs/*.h) ramn_test_vectors.h $(CORE)/Src/ramn_telematics.c
	$(CC) $(CFLAGS) -DSECOC_LINK_ROLE_SENDER -o $@ $(SRCS)

$(BIN_A): $(SRCS_A) $(wildcard stubs/*.h) ramn_test_vectors.h $(CORE)/Src/ramn_screen_image.c
	$(CC) $(CFLAGS_A) -o $@ $(SRCS_A)

clean:
	rm -f $(BIN) $(BIN_A)

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
