# Host-side tests: compile hardware-independent firmware sources against stub
# ESP-IDF headers and drive them with a fake clock. No ESP-IDF or hardware is
# needed.
#
#   make          build and run every test
#   make build    compile only
#   make clean
#
# Adding a test: drop test_<name>.c in this directory; it is discovered
# automatically, built as its own binary, and run by `make`. See README.md.
#
# The precondition suite is also built a second time with -DCAN_BUS_COUNT=1
# (see SINGLE_BUS_BINS below) so the v300/single-bus preprocessor branches are
# exercised rather than merely compiled out.
#
# Firmware sources under test are copied into build/src first: for quoted
# #includes the compiler searches the including file's own directory before
# any -I path, so compiling them in place inside main/ would pick up the real
# ESP-IDF headers there (can.h, config_server.h, ...) instead of the stubs.

MAIN := ../../main

# firmware files tests compile against (copied into build/src)
FW_FILES := utf8_utf16_converter.c utf8_utf16_converter.h hsm.c hsm.h \
	isotp_tx.c isotp_tx.h track_popup.c track_popup.h \
	precondition.c precondition.h persistent_settings.c persistent_settings.h
# firmware sources linked into every test binary (tests that #include a
# source instead, for introspection, don't list it here)
LINK_SRCS := build/src/hsm.c

CC ?= cc
CFLAGS ?= -std=gnu17 -Wall -Wextra -Wno-unused-parameter -g -pthread
CPPFLAGS += -Istubs -Isupport -Ibuild/src

# every test_*.c, plus the single-bus build of the precondition suite: same
# source, compiled for a one-bus board, so its CAN_BUS_COUNT fallbacks run in CI
SINGLE_BUS_BINS := build/test_precondition_1bus
TEST_BINS := $(patsubst %.c,build/%,$(wildcard test_*.c)) $(SINGLE_BUS_BINS)
FW_COPIES := $(addprefix build/src/,$(FW_FILES))
HEADERS := $(wildcard stubs/*.h stubs/*/*.h support/*.h)

.PHONY: all build run clean

all: run

build: $(TEST_BINS)

run: $(TEST_BINS)
	@rc=0; \
	for t in $(TEST_BINS); do \
		echo "=== $$t"; \
		./$$t || rc=1; \
	done; \
	exit $$rc

build/src/%: $(MAIN)/% | build/src
	cp $< $@

build/src:
	mkdir -p build/src

build/test_%: test_%.c $(FW_COPIES) $(HEADERS) | build/src
	$(CC) $(CFLAGS) $(CPPFLAGS) -DEXPECT_CAN_BUS_COUNT=2 -o $@ $< $(LINK_SRCS) \
		$(if $(filter test_isotp_tx.c test_track_popup.c,$<),build/src/isotp_tx.c) \
		$(if $(filter test_track_popup.c,$<),build/src/utf8_utf16_converter.c)

# Explicit rule, so it wins over the build/test_% pattern above: the same suite
# with the one-bus board's CAN_BUS_COUNT. The stub in stubs/can.h is #ifndef'd
# so this -D takes effect.
build/test_precondition_1bus: test_precondition.c $(FW_COPIES) $(HEADERS) | build/src
	$(CC) $(CFLAGS) $(CPPFLAGS) -DCAN_BUS_COUNT=1 -DEXPECT_CAN_BUS_COUNT=1 -o $@ $< $(LINK_SRCS)

clean:
	rm -rf build
