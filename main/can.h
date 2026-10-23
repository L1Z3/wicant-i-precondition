/*
 * The slice of the wicant-i-precondition firmware's can.h that precondition.c
 * uses, implemented on can_manager. See main_precondition.c.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "driver/twai.h"

/* The precondition code names the bus a frame came from or goes to. The Pro
 * has the one, on the ESP's TWAI controller; CAN_BUS_1 only exists on the
 * dual-bus board and nothing here sends on it. */
typedef enum {
	CAN_BUS_0 = 0,
	CAN_BUS_1 = 1,
} can_bus_t;

/* precondition.c, beep.c and friends build their single-bus paths from this:
   the head unit is reached on CAN_BUS_0 too, and nothing is bridged */
#define CAN_BUS_COUNT 1

esp_err_t can_send(can_bus_t bus, twai_message_t *message, TickType_t ticks_to_wait);
