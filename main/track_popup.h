#ifndef __TRACK_POPUP_H__
#define __TRACK_POPUP_H__

#include <stdbool.h>
#include "hsm.h"

// Initialize the cluster track-selection popup service. Reuse a known active
// media type and transport when possible, otherwise force the fallback type.
void track_popup_init(void);

// Drive the popup state machine from the precondition machine's global hooks.
void track_popup_tick(void);
void track_popup_rx(const twai_message_t *msg, can_bus_t rx_bus);
fwd_result_t track_popup_fwd(twai_message_t *msg, can_bus_t fwd_bus);

// Queue arbitrary UTF-8 text for the cluster's track-selection popup. The
// text is converted to UTF-16LE and copied before this function returns.
// Returns false if uninitialized, for invalid/oversized text, or a full queue.
bool track_popup_show(const char *utf8_text);

#endif
