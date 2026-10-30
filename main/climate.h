#ifndef __CLIMATE_H__
#define __CLIMATE_H__

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "hsm.h"

// Remote climate start (prototype, second-bus hardware only).
//
// Replays the head unit's own remote-climate activation on the car bus and
// blocks the head unit's copies of the two frames that carry it while the
// replay runs. Driven from the precondition machine's global hooks; the
// sequence itself is single-writer on the precondition task.

// Drive the sequence from the precondition machine's global tick hook.
void climate_tick(void);

// Block the head unit's 0x4A2/0x4F1 copies on their way to the car while a
// request is queued or a sequence is running.
fwd_result_t climate_fwd_hook(twai_message_t *msg, can_bus_t fwd_bus);

// Queue a remote climate start. Returns ESP_ERR_NOT_SUPPORTED on hardware
// without the second bus and ESP_ERR_INVALID_STATE while one is already
// running. Blocking starts as soon as this returns ESP_OK, not when the
// precondition task picks the request up.
esp_err_t climate_request_start(void);

// Whether a start is queued or running; the web UI disables its button.
bool climate_is_active(void);

// Feed every received frame in so the module can log what the car answers.
// Cheap for the frames it does not track, safe to call from the CAN task.
void climate_rx_hook(const twai_message_t *msg, can_bus_t rx_bus);

// One line for the Status tab: what the replay is doing and the last frame
// that looked like a response, e.g.
//   "idle; 0x4EB 80 hu +1.2 s"
// Returns false on hardware without the second bus (nothing is written then).
bool climate_get_status(char *out, size_t len);

#endif
