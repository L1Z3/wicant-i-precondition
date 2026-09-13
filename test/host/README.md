# Host-side tests

Plain-C tests that compile the hardware-independent firmware sources
(`main/hsm.c`, `main/precondition.c`, and `main/persistent_settings.c`) against
stub ESP-IDF headers and drive them with a fake clock and fake CAN traffic.
They need no ESP-IDF install and no hardware to run:

```sh
make -C test/host
```

CI runs them as the `host-tests` job of the build-firmware workflow.

## Layout

```
test/host/
  Makefile              discovers and runs every test_*.c
  support/
    test_support.h      fake clock, CHECK/CHECK_MSG, test_report()
  stubs/                minimal stand-ins for ESP-IDF / firmware headers
    can.h  config_server.h  esp_log.h  esp_timer.h  nvs.h
    driver/twai.h  freertos/FreeRTOS.h  freertos/queue.h  freertos/task.h
  test_hsm.c            state machine engine semantics
  test_precondition.c   precondition features (built for two bus counts)
```

## How it works

The Makefile copies the firmware sources listed in `FW_FILES` into
`build/src/` and compiles there with `-Istubs`. The copy step matters: for
quoted `#include`s the compiler searches the including file's own directory
first, so building `precondition.c` in place inside `main/` would resolve
`can.h` and `config_server.h` to the real headers instead of the stubs. 
Copies are dependency-tracked, so editing a file under `main/` rebuilds the
tests against it.

Each `test_*.c` is one self-contained test program: it must run everything it
covers and exit 0 on success, non-zero on failure.

`test_precondition.c` is the exception to one-binary-per-source: `precondition.c`
picks the head-unit bus from `CAN_BUS_COUNT` (on a single-bus board
`HEAD_UNIT_BUS` falls back to `CAN_BUS_0`), so the suite is built twice —
`build/test_precondition` for the two-bus custom board and
`build/test_precondition_1bus` with `-DCAN_BUS_COUNT=1` for the v300 board — and
both run under `make`. Each build passes its configuration as
`-DEXPECT_CAN_BUS_COUNT=<n>` and `run_suite()` asserts it, because the suite is
self-consistent under either value: without that marker a stub or flag
regression could run the same configuration twice and leave the single-bus
branches uncovered.

## Adding a test

1. Create `test_<name>.c` here. Include `support/test_support.h` exactly once
   for the fake clock (`fake_now`, backing the `esp_timer_get_time()` stub),
   `CHECK`/`CHECK_MSG`, and `test_report()` (returns the exit code).
   `test_support.h` defines symbols, not just declares them, which is safe
   because each test binary contains a single test translation unit.
2. Either link firmware sources (like `test_hsm.c`; they are listed in
   `LINK_SRCS`) or `#include` one directly (like `test_precondition.c`
   includes `persistent_settings.c` and `precondition.c`) when the test needs
   to reach static state for introspection.
3. If the source under test isn't copied yet, add it to `FW_FILES` in the
   Makefile. If it pulls in a new ESP-IDF header, add a minimal stub under
   `stubs/`. Only what the code under test actually touches, and any
   constants mirrored from real headers must stay in sync (see
   `stubs/config_server.h`). Stubs shared by several tests live in `stubs/`;
   fakes specific to one test (like the recording `can_send`) live in that
   test file. `stubs/can.h` supplies `CAN_BUS_COUNT` (2, under `#ifndef`)
   because the real header takes it from `hw_config.h`; leave it undefined and
   `#if CAN_BUS_COUNT > 1` silently compiles the single-bus side. A test needing
   the other configuration needs its own explicit rule, as
   `build/test_precondition_1bus` has.

## Quirks worth knowing

- `test_precondition` forks a child per suite: the firmware's config snapshot,
  repeating-mode latch, and platform discovery flags live in process static
  storage, so each mode/press combination needs a fresh process. Run one suite directly with
  `build/test_precondition <name substring>` (e.g. `continuous`).
- The `nvs.h` stub is an in-memory single-slot fake; tests seed and inspect
  it through `fake_nvs_exists`/`fake_nvs_value` (see the persistent-restore
  suite).
- The `esp_log.h` stub prints to stdout, so test output includes the engine's
  transition log.
