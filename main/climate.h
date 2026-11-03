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
// (I-CAN) where the telematics unit emits 0x39B with a class byte: 0x01
// lock/unlock, 0x02 charge limit, 0x03 climate. The car's command parameters
// live on M-CAN, but a M-CAN-only replay of the climate start was ignored by
// the car in every test so far, so which of these actually work without the
// I-CAN session is exactly what these buttons are for.

// 0 = climate start, 1 = start + front defrost, 2 = start + rear/side
// defrost, 3 = start + steering wheel heat, 4 = climate stop, 5 = lock,
// 6 = unlock.
esp_err_t climate_request_start(uint8_t command);

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
