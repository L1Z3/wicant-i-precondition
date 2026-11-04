#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "hsm.h"

// Remote commands off the head unit's own bus (M-CAN), replayed by the kit.
//
// The commands are the ones observed in the Dual_* captures: two climate
// starts with extra functions, the plain climate start, the climate stop, and
// the door lock/unlock. The charge-limit commands also live on M-CAN (0x4C5)
// but are deliberately not implemented here.
//
// Every one of these commands opens a session on the head unit's second bus
// (I-CAN) where the telematics unit emits 0x39B: a state byte, the cabin
// setpoint in byte 1, and a class byte (0x01 lock/unlock, 0x02 charge limit,
// 0x03 climate). The command itself is on M-CAN, and this module replays the
// 0x39B setpoint alongside it because the car applies the temperature it is
// told, not the one in the 0x4A2 request (which carries none).

// 0 = climate start, 1 = start + front defrost, 2 = start + rear/side
// defrost, 3 = start + steering wheel heat, 4 = climate stop, 5 = lock,
// 6 = unlock, 7 = climate start with the 3 s wake preamble for a sleeping car.
//
// climate_request_start() announces the default 22.0 C. The car takes the
// setpoint from 0x39B byte 1 (raw = 2T - 28) and its mirror 0x2AC byte 4, so
// climate_request_start_temp() is the entry point that names a temperature:
// tenths of a degree C, 140..320, e.g. 225 = 22.5 C.
esp_err_t climate_request_start(uint8_t command);
esp_err_t climate_request_start_temp(uint8_t command, int16_t setpoint_tenths);

#define CLIMATE_SETPOINT_MIN_TENTHS 140   // 14.0 C
#define CLIMATE_SETPOINT_MAX_TENTHS 320   // 32.0 C
#define CLIMATE_SETPOINT_DEFAULT_TENTHS 220

uint8_t climate_command_count(void);
const char *climate_command_label(uint8_t command);

bool climate_is_active(void);

// Called from the precondition tick (40 ms) and from the CAN receive path.
void climate_tick(void);
void climate_rx_hook(const twai_message_t *msg, can_bus_t rx_bus);

// Blocks the head unit's copies of the command frames while a replay runs, so
// the car sees one command and not two interleaved ones.
fwd_result_t climate_fwd_hook(twai_message_t *msg, can_bus_t fwd_bus);

// One line for the Status tab: phase plus the last frame the car answered with.
bool climate_get_status(char *out, size_t len);
