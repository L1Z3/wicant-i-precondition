// host-test stub for main/can.h: the real one drags in hw_config.h and the
// ESP-IDF driver headers. tests provide their own can_send (e.g. a recorder).
#pragma once
#include <stdint.h>
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"

// The real value comes from hw_config.h per variant. The stub models the
// two-bus custom board, because that is the configuration the tests' bus
// expectations assume: with this undefined, `#if CAN_BUS_COUNT > 1` would
// silently compile the single-bus fallbacks instead. `#ifndef` so the
// single-bus binary can override it with -DCAN_BUS_COUNT=1 (see Makefile).
#ifndef CAN_BUS_COUNT
#define CAN_BUS_COUNT 2
#endif

typedef enum { CAN_BUS_0 = 0, CAN_BUS_1 = 1 } can_bus_t;
typedef int esp_err_t;

esp_err_t can_send(can_bus_t bus, twai_message_t *message, TickType_t ticks_to_wait);
