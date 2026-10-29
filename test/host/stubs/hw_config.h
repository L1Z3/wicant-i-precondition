// host-test stub for main/hw_config.h: only the feature macros the firmware
// sources under test read. The real header is per-variant and pulls in the
// ESP-IDF pin/ADC/SPI headers, which the host build has no use for.
#pragma once
#include "can.h"

// The custom board carries the second (MCP2515) CAN bus, the v300 does not, so
// the host builds derive it from the same knob CAN_BUS_COUNT uses.
// `#ifndef` so a single-bus binary can still override it.
#ifndef HW_HAS_MCP2515
#define HW_HAS_MCP2515 (CAN_BUS_COUNT > 1)
#endif
