// host-test stub for main/can.h: the real one drags in hw_config.h and the
// ESP-IDF driver headers. tests provide their own can_send (e.g. a recorder).
#pragma once
#include <stdint.h>
#include "esp_err.h"
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

// bitrate indices, as in main/can.h
#define CAN_5K   0
#define CAN_10K  1
#define CAN_20K  2
#define CAN_25K  3
#define CAN_50K  4
#define CAN_100K 5
#define CAN_125K 6
#define CAN_250K 7
#define CAN_500K 8
#define CAN_800K 9
#define CAN_1000K 10
#define CAN_AUTO 11

esp_err_t can_send(can_bus_t bus, twai_message_t *message, TickType_t ticks_to_wait);
void can_enable(can_bus_t bus);
void can_disable(can_bus_t bus);
void can_set_silent(can_bus_t bus, uint8_t flag);
void can_set_loopback(can_bus_t bus, uint8_t flag);
void can_set_bitrate(can_bus_t bus, uint8_t rate);
void can_set_filter(can_bus_t bus, uint32_t f);
void can_set_mask(can_bus_t bus, uint32_t m);
