#ifndef __TRACK_POPUP_H__
#define __TRACK_POPUP_H__

#include <stdbool.h>
#include "hsm.h"

// The cluster payload holds up to 50 UTF-16 code units. A matching valid UTF-8
// string needs at most three bytes per UTF-16 code unit:
// https://stackoverflow.com/a/58581109
#define TRACK_POPUP_MAX_TEXT_CODE_UNITS 50U
#define TRACK_POPUP_MAX_TEXT_UTF8_BYTES \
    (TRACK_POPUP_MAX_TEXT_CODE_UNITS * 3U)

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

// Queue a popup with a severity marker prefixed to the supplied text.
bool track_popup_show_info(const char *utf8_text);
bool track_popup_show_warning(const char *utf8_text);
bool track_popup_show_error(const char *utf8_text);

#endif
