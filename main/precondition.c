#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "esp_timer.h"
#include "can.h"
#include "precondition.h"

// ********************* 0x4E8 distance/flag display *********************

typedef enum {
    DIST_UNIT_M = 0x0U,
    DIST_UNIT_KM = 0x1U,
    DIST_UNIT_MI = 0x2U,
    DIST_UNIT_FT = 0x3U,
    DIST_UNIT_YD = 0x4U,
} dist_unit_t;

typedef enum {
    FLAG_DESTINATION = 0x0U,
    FLAG_BLUE_1 = 0x1U,
    FLAG_BLUE_2 = 0x2U,
    FLAG_BLUE_3 = 0x3U,
    FLAG_BLUE_4 = 0x4U,
    FLAG_NONE = 0xFU,
} flag_type_t;

// Set data bytes on a 0x4E8 CANPacket_t to display a distance and flag.
//   distance_int: integer part (0-65534, or 0xFFFF to hide number/unit)
//   distance_tenths: tenths digit (0-9, only shown when unit is km or mi and integer < 100)
//   unit: distance unit (see dist_unit_t)
//   flag: flag icon (see flag_type_t)
static void set_0x4e8_distance_flag(twai_message_t *packet, uint16_t distance_int, uint8_t distance_tenths, dist_unit_t unit, flag_type_t flag) {
    packet->data[0] = (uint8_t)(((distance_tenths & 0xFU) << 4U) | (unit & 0xFU));
    packet->data[4] = (uint8_t)(distance_int & 0xFFU);
    packet->data[5] = (uint8_t)((distance_int >> 8U) & 0xFFU);
    packet->data[6] = (packet->data[6] & 0xF0U) | (flag & 0xFU);
}

// ********************* precondition logic *********************

// is the user currently requesting preconditioning to be active?
static bool precondition_requested = false;
// timestamp of when the user requested preconditioning
static uint32_t precondition_requested_ts = 0U;
// timestamp of last attempt to send precondition message, used for retry logic
static uint32_t precondition_last_attempt_ts = 0U;
// number of ticks remaining to send precondition start/stop messages, used for initial burst logic
static uint8_t precondition_start_ticks_remaining = 0U;
static uint8_t precondition_stop_ticks_remaining = 0U;
// number of times we've retried sending precondition messages, used for retry logic
static uint8_t precondition_retries = 0U;
// has preconditioning been confirmed to be starting by the 2AD status frame?
static bool precondition_starting_confirmed = false;
// has preconditioning been confirmed to be active by the 2AD status frame?
static bool precondition_started_confirmed = false;
// has precondition stop been confirmed by the 2AD status frame?
static bool precondition_stop_confirmed = true;
// track previous state of star button for edge detection
static bool star_button_prev = false;

#define PRECONDITION_DEBOUNCE_US 1000000U  // 1 second
#define PRECONDITION_START_PHASE1_TICKS 3U // 4003 message
#define PRECONDITION_START_PHASE2_TICKS 3U // E007 message
#define PRECONDITION_START_TICKS (PRECONDITION_START_PHASE1_TICKS + PRECONDITION_START_PHASE2_TICKS)
#define PRECONDITION_STOP_PHASE1_TICKS 3U  // 0000 message
#define PRECONDITION_STOP_PHASE2_TICKS 3U  // E007 message
#define PRECONDITION_STOP_TICKS (PRECONDITION_STOP_PHASE1_TICKS + PRECONDITION_STOP_PHASE2_TICKS)
#define PRECONDITION_RETRY_US 10000000U  // 10 seconds
#define PRECONDITION_MAX_RETRIES 4U
#define PRECONDITION_STARTED_TIMEOUT_US 70000000U  // 70 seconds

#define SECONDS_UNTIL_START(elapsed) \
    (((elapsed) >= PRECONDITION_STARTED_TIMEOUT_US) ? 0U : \
     ((PRECONDITION_STARTED_TIMEOUT_US - (elapsed)) / 1000000U))

#define SECONDS_UNTIL_STOP_RETRY(elapsed) \
    (((elapsed) >= PRECONDITION_RETRY_US) ? 0U : \
     ((PRECONDITION_RETRY_US - (elapsed)) / 1000000U))

static uint32_t now_us(void) {
    return (uint32_t)esp_timer_get_time();
}

static uint32_t ts_elapsed(uint32_t now, uint32_t old) {
    return now - old;
}

static void send_precondition_start_msg(uint8_t ticks_remaining) {
    twai_message_t packet = {0};
    packet.identifier = 0x0C7U;
    packet.data_length_code = 8U;
    if (ticks_remaining > PRECONDITION_START_PHASE2_TICKS) {
        // send 0000004003000000 to 0x0C7
        packet.data[3] = 0x40U;
        packet.data[4] = 0x03U;
    } else {
        // send 000000E007000000 to 0x0C7
        packet.data[3] = 0xE0U;
        packet.data[4] = 0x07U;
    }
    // TODO(ejones): ensure that blocking for 1 tick is the right move here and elsewhere
    can_send(&packet, 1);
}

static void send_precondition_stop_msg(uint8_t ticks_remaining) {
    twai_message_t packet = {0};
    packet.identifier = 0x0C7U;
    packet.data_length_code = 8U;
    if (ticks_remaining <= PRECONDITION_STOP_PHASE2_TICKS) {
        // send 000000E007000000 to 0x0C7
        packet.data[3] = 0xE0U;
        packet.data[4] = 0x07U;
    }
    can_send(&packet, 1);
}

// Decide whether to block, modify, or passthrough a message for preconditioning.
// Modifies packet data in-place when returning FWD_MODIFIED.
// When NO_MITM is defined, this only has an effect when FWD_MODIFIED is returned
fwd_result_t precondition_fwd_hook(twai_message_t *to_send) {
    // block 0x0C7 message so that the head unit doesn't turn off preconditioning on us
    // (no effect currently since we're doing NO_MITM path)
    // if (precondition_requested && to_send->identifier == 0x0C7U) {
    //     return FWD_BLOCK;
    // }

    // MITM 0x4ED message while preconditioning is requested
    if (precondition_requested && to_send->identifier == 0x4EDU) {
        to_send->data[5] = 0x10U;
        to_send->data[6] = 0xA0U;
        to_send->data[7] = 0x00U;
        return FWD_MODIFIED;
    }

    // we are currently starting preconditioning and want to display the countdown flag.
    if (precondition_requested && !precondition_started_confirmed) {
        uint32_t now = now_us();
        uint32_t time_since_last_attempt = ts_elapsed(now, precondition_last_attempt_ts);
        if (to_send->identifier == 0x4E8U) {
            set_0x4e8_distance_flag(
                to_send,
                SECONDS_UNTIL_START(time_since_last_attempt),
                // display retry count in tenths digit
                precondition_retries % 10,
                precondition_retries == 0 ? DIST_UNIT_YD : DIST_UNIT_KM,
                // switch to destination flag after we get 05 for 2AD
                precondition_starting_confirmed ? FLAG_DESTINATION : FLAG_BLUE_1
            );
            return FWD_MODIFIED;
        }

        if (to_send->identifier == 0x4CCU) {
            to_send->data[0] = 0x02U;
            return FWD_MODIFIED;
        }
    }

    // we are currently trying to stop preconditioning and want to display the retry status.
    if (!precondition_requested && !precondition_stop_confirmed && precondition_retries < PRECONDITION_MAX_RETRIES) {
        uint32_t now = now_us();
        uint32_t time_since_last_attempt = ts_elapsed(now, precondition_last_attempt_ts);
        if (to_send->identifier == 0x4E8U) {
            set_0x4e8_distance_flag(
                to_send,
                SECONDS_UNTIL_STOP_RETRY(time_since_last_attempt),
                // display retry count in tenths digit
                precondition_retries % 10,
                precondition_retries == 0 ? DIST_UNIT_FT : DIST_UNIT_MI,
                FLAG_NONE
            );
            return FWD_MODIFIED;
        }

        if (to_send->identifier == 0x4CCU) {
            to_send->data[0] = 0x02U;
            return FWD_MODIFIED;
        }
    }

    // otherwise, passthrough without modification
    return FWD_PASSTHROUGH;
}

static void start_preconditioning(uint32_t now) {
    precondition_requested = true;
    precondition_requested_ts = now;
    precondition_last_attempt_ts = now;
    precondition_start_ticks_remaining = PRECONDITION_START_TICKS;
    precondition_starting_confirmed = false;
    precondition_started_confirmed = false;
    precondition_retries = 0U;
    // HACK: disable retries
    precondition_starting_confirmed = true;
}

static void stop_preconditioning(uint32_t now) {
    precondition_requested = false;
    precondition_last_attempt_ts = now;
    precondition_stop_ticks_remaining = PRECONDITION_STOP_TICKS;
    precondition_stop_confirmed = false;
    precondition_retries = 0U;
    // HACK: disable retries
    precondition_stop_confirmed = true;
}

void precondition_can_rx_hook(twai_message_t *to_push) {
    // 0x2AD status frame: second byte indicates precondition state
    //   0x01 = off/idle, 0x05 = starting, 0x15 = fully running
    // HACK: disable retries
    // if (to_push->identifier == 0x2ADU) {
    //     uint8_t status = to_push->data[1];
    //     if (precondition_requested && !precondition_starting_confirmed) {
    //         if (status == 0x05U || status == 0x15U) {
    //             precondition_starting_confirmed = true;
    //         }
    //     }
    //     if (precondition_requested && !precondition_started_confirmed) {
    //         if (status == 0x15U) {
    //             precondition_started_confirmed = true;
    //         }
    //     }
    //     if (precondition_requested && precondition_started_confirmed) {
    //         if (status == 0x05U) {
    //             // preconditioning was previously fully active, but now it's only showing as starting.
    //             // this is a weird situation to be in; let's just reset the current attempt time,
    //             // and let the retry logic continue as normal if it doesn't resolve itself after a while
    //             precondition_last_attempt_ts = now_us();
    //             precondition_started_confirmed = false;
    //         }
    //         if (status == 0x01U) {
    //             // preconditioning was previously fully active, but now it's showing as off.
    //             // it's possible that the car has reached the (rare) "Precondition complete" state.
    //             // until we have a better way to distinguish that state from a real failure mode (TODO(ejones)),
    //             // let's just assume everything is fine and reset our state
    //             uint32_t now = now_us();
    //             stop_preconditioning(now);
    //             precondition_stop_ticks_remaining = 0U;
    //             precondition_stop_confirmed = true;
    //         }
    //     }
    //     if (!precondition_requested && !precondition_stop_confirmed && precondition_stop_ticks_remaining == 0U) {
    //         if (status == 0x01U) {
    //             precondition_stop_confirmed = true;
    //         }
    //     }
    // }

    // 0x448 has button presses:
    // 7F 00 00 00 00 00 00 00 -> Idle
    // 15 00 00 01 00 00 00 00 -> Mute
    // 67 00 00 00 00 10 00 00 -> Star
    // 31 00 00 00 04 00 00 00 -> Menu
    // 7A 00 04 00 00 00 00 00 -> speak
    // listen for star button press (rising edge only), and toggle preconditioning
    if (to_push->identifier == 0x448U) {
        bool star_button = (to_push->data[5] == 0x10U);
        if (star_button && !star_button_prev) {
            uint32_t now = now_us();
            if (!precondition_requested) {
                start_preconditioning(now);
            } else if (ts_elapsed(now, precondition_requested_ts) > PRECONDITION_DEBOUNCE_US) {
                stop_preconditioning(now);
            }
        }
        star_button_prev = star_button;
    }
}

// called every 40ms
void precondition_tick(void) {
    uint32_t now = now_us();
    uint32_t time_since_last_attempt = ts_elapsed(now, precondition_last_attempt_ts);

    // give up and send one stop attempt if start retries exhausted
    if (precondition_requested
            && precondition_start_ticks_remaining == 0U
            && precondition_retries >= PRECONDITION_MAX_RETRIES
            && ((!precondition_starting_confirmed && time_since_last_attempt > PRECONDITION_RETRY_US)
                || (!precondition_started_confirmed && time_since_last_attempt > PRECONDITION_STARTED_TIMEOUT_US))) {
        // HACK: disable retries
        // stop_preconditioning(now);
        // precondition_retries = PRECONDITION_MAX_RETRIES;  // don't retry the stop; this is a failure case already
    }

    // retry start if not confirmed to be starting yet and it's been long enough
    if (precondition_requested && !precondition_starting_confirmed
            && precondition_start_ticks_remaining == 0U
            && precondition_retries < PRECONDITION_MAX_RETRIES
            && time_since_last_attempt > PRECONDITION_RETRY_US) {
        // HACK: disable retries
        // precondition_last_attempt_ts = now;
        // precondition_start_ticks_remaining = PRECONDITION_START_TICKS;
        // precondition_retries++;
    }

    // retry start if not confirmed to be started yet and it's been long enough (i.e. we got 2AD 05 but not 15 after a long time)
    if (precondition_requested && !precondition_started_confirmed
            && precondition_start_ticks_remaining == 0U
            && precondition_retries < PRECONDITION_MAX_RETRIES
            && time_since_last_attempt > PRECONDITION_STARTED_TIMEOUT_US) {
        // HACK: disable retries
        precondition_started_confirmed = true;
        // precondition_last_attempt_ts = now;
        // precondition_start_ticks_remaining = PRECONDITION_START_TICKS;
        // precondition_retries++;
    }

    // retry stop if not confirmed
    if (!precondition_requested && !precondition_stop_confirmed
            && precondition_stop_ticks_remaining == 0U
            && precondition_retries < PRECONDITION_MAX_RETRIES
            && time_since_last_attempt > PRECONDITION_RETRY_US) {
        // HACK: disable retries
        // precondition_last_attempt_ts = now;
        // precondition_stop_ticks_remaining = PRECONDITION_STOP_TICKS;
        // precondition_retries++;
    }

    // send initial burst of start messages
    if (precondition_requested && precondition_start_ticks_remaining > 0U) {
        send_precondition_start_msg(precondition_start_ticks_remaining);
        precondition_start_ticks_remaining--;
    }

    // send initial burst of stop messages
    if (!precondition_requested && precondition_stop_ticks_remaining > 0U) {
        send_precondition_stop_msg(precondition_stop_ticks_remaining);
        precondition_stop_ticks_remaining--;
    }
}
