#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "can.h"
#include "hsm.h"
#include "precondition.h"
#include "persistent_settings.h"
#include "config_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define TAG __func__

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

// ********************* activation buttons *********************

typedef enum {
    // frame carries the current button state: pressed while (byte & mask) == value
    MSG_STATE,
    // frame is an event whose masked byte takes distinct press/release values
    MSG_EVENT,
} message_type_t;

typedef struct {
    uint32_t frame_id;
    message_type_t type;
    union {
        struct {
            uint8_t byte_index;
            uint8_t byte_mask;
            uint8_t byte_value;
        } state;
        struct {
            uint8_t byte_index;
            uint8_t byte_mask;
            // TODO(ejones): maybe use a bit set or some other
            // way of representing any number of possibilities
            uint8_t press_values[2];   // either value means pressed
            uint8_t release_values[2]; // either value means released
        } pair;
    };
} message_payload_t;

// map of the buttons that can be used to activate preconditioning.
// note: SW buttons (0x448) have a periodic idle message;
//       AVN buttons (0x651/0x652) only send on press/release.
static const message_payload_t activation_messages[NUM_PRECON_BUTTONS] = {
    [SW_STAR]         = {0x448, MSG_STATE, .state = {5, 0xF0, 0x10}},
    [AVN_STAR]        = {0x652, MSG_EVENT,  .pair  = {1, 0x0F, {0x04, 0x07}, {0x00, 0x03}}},
    [AVN_TUNER_IN]    = {0x651, MSG_EVENT,  .pair  = {3, 0xF0, {0x40, 0x70}, {0x00, 0x30}}},
    [AVN_VOL_IN]      = {0x651, MSG_EVENT,  .pair  = {1, 0xF0, {0x40, 0x70}, {0x00, 0x30}}},
    [SW_MODE]         = {0x448, MSG_STATE, .state = {2, 0xF0, 0x40}},
    [SW_SPEAK]        = {0x448, MSG_STATE, .state = {2, 0x0F, 0x01}},
    [SW_CALL]         = {0x448, MSG_STATE, .state = {2, 0x0F, 0x04}},
    [SW_VOL_IN]       = {0x448, MSG_STATE, .state = {3, 0x0F, 0x01}},
    [SW_VOL_UP]       = {0x448, MSG_STATE, .state = {4, 0x0F, 0x01}},
    [SW_VOL_DOWN]     = {0x448, MSG_STATE, .state = {3, 0xF0, 0x40}},
    [SW_SKIP_UP]      = {0x448, MSG_STATE, .state = {3, 0xF0, 0x10}},
    [SW_SKIP_DOWN]    = {0x448, MSG_STATE, .state = {3, 0x0F, 0x04}},
    [SW_OK]           = {0x448, MSG_STATE, .state = {6, 0xF0, 0x10}},
    [AVN_MAP]         = {0x652, MSG_EVENT,  .pair  = {0, 0xF0, {0x40, 0x70}, {0x00, 0x30}}},
    [AVN_NAV]         = {0x652, MSG_EVENT,  .pair  = {0, 0xF0, {0x10, 0xD0}, {0x00, 0xC0}}},
    [AVN_MEDIA]       = {0x652, MSG_EVENT,  .pair  = {0, 0x0F, {0x01, 0x0D}, {0x00, 0x0C}}},
    [AVN_TUNER_UP]    = {0x652, MSG_EVENT,  .pair  = {3, 0x0F, {0x04, 0x07}, {0x00, 0x03}}},
    [AVN_TUNER_DOWN]  = {0x652, MSG_EVENT,  .pair  = {3, 0x0F, {0x01, 0x0D}, {0x00, 0x0C}}},
    [EV6_AVN_SETUP]   = {0x652, MSG_EVENT,  .pair  = {1, 0x0F, {0x01, 0x0D}, {0x00, 0x0C}}},
};
_Static_assert(sizeof(activation_messages) / sizeof(activation_messages[0])
               == NUM_PRECON_BUTTONS, "button table size mismatch");

static bool state_matches(const message_payload_t *msg, const twai_message_t *f) {
    return (f->data[msg->state.byte_index] & msg->state.byte_mask) == msg->state.byte_value;
}

static bool pair_matches(const message_payload_t *msg, const twai_message_t *f, const uint8_t values[2]) {
    uint8_t masked = f->data[msg->pair.byte_index] & msg->pair.byte_mask;
    return masked == values[0] || masked == values[1];
}

static bool activation_is_press(const message_payload_t *msg, const twai_message_t *f) {
    if (f->identifier != msg->frame_id) {
        return false;
    }
    switch (msg->type) {
        case MSG_STATE:
            return state_matches(msg, f);
        case MSG_EVENT:
            return pair_matches(msg, f, msg->pair.press_values);
    }
    return false;
}

static bool activation_is_release(const message_payload_t *msg, const twai_message_t *f) {
    if (f->identifier != msg->frame_id) {
        return false;
    }
    switch (msg->type) {
        case MSG_STATE:
            return !state_matches(msg, f);
        case MSG_EVENT:
            return pair_matches(msg, f, msg->pair.release_values);
    }
    return false;
}

// ********************* frame ids and timing constants *********************

// 0x2AD on Ioniq 5/EV6, 0x0A82AA03 on Ioniq 6
#define IS_STATUS_FRAME(frame_id) \
    ((frame_id) == 0x2ADU || (frame_id) == 0x0A82AA03U)

#define STATUS_MASK 0b00011111U  // i.e. 0x15 and 0x55 are both valid "started" status
#define STATUS_IDLE(status_byte) \
    (((status_byte) & STATUS_MASK) == 0x01U)
#define STATUS_STARTING(status_byte) \
    (((status_byte) & STATUS_MASK) == 0x05U)
#define STATUS_STARTED(status_byte) \
    (((status_byte) & STATUS_MASK) == 0x15U)

#define IS_POWER_STATUS_FRAME(frame_id) \
    ((frame_id) == 0x038U)

// TODO(ejones): unclear if this mask/value is necessary and sufficient
#define POWER_STATUS_MASK 0x0FU
#define POWER_STATUS_READY(power_status_byte) \
    (((power_status_byte) & POWER_STATUS_MASK) == 0x04U)
#define POWER_STATUS_CHARGING(power_status_byte) \
    (((power_status_byte) & POWER_STATUS_MASK) == CAR_POWER_CHARGING)

#define PRECONDITION_DEBOUNCE_US 1000000U  // 1 second
#define PRECONDITION_LONG_PRESS_US 1000000U  // short/long press threshold: 1 second
#define PRECONDITION_START_PHASE1_TICKS 3U // 4003 message
#define PRECONDITION_START_PHASE2_TICKS 3U // E007 message
#define PRECONDITION_START_TICKS (PRECONDITION_START_PHASE1_TICKS + PRECONDITION_START_PHASE2_TICKS)
#define PRECONDITION_STOP_PHASE1_TICKS 3U  // 0000 message
#define PRECONDITION_STOP_PHASE2_TICKS 3U  // E007 message
#define PRECONDITION_STOP_TICKS (PRECONDITION_STOP_PHASE1_TICKS + PRECONDITION_STOP_PHASE2_TICKS)
#define PRECONDITION_RETRY_US 10000000U  // 10 seconds
#define PRECONDITION_MAX_RETRIES 4U
#define PRECONDITION_STARTED_TIMEOUT_US 70000000U  // 70 seconds
#define PRECONDITION_CAR_START_DELAY_US 8000000U  // persistent mode: wait 8 seconds after READY before relaunching
#define REPEATING_MODE_RETRY_INTERVAL_US (5LL * 60LL * 1000000LL)  // 5 minutes between re-nudges

#define BATTERY_TEMPERATURE_FRAME_ID 0x152U
#define BATTERY_TEMPERATURE_MIN_INDEX 0U
#define BATTERY_TEMPERATURE_MAX_INDEX 1U
#define BATTERY_TEMPERATURE_DATA_LENGTH 2U

#define IS_BATTERY_TEMPERATURE_FRAME(frame_id) ((frame_id) == BATTERY_TEMPERATURE_FRAME_ID)

#define CAR_BUS CAN_BUS_0
#define HEAD_UNIT_BUS CAN_BUS_1

#define SECONDS_UNTIL_START(elapsed) \
    (((elapsed) >= PRECONDITION_STARTED_TIMEOUT_US) ? 0U : \
     ((PRECONDITION_STARTED_TIMEOUT_US - (elapsed)) / 1000000U))

#define SECONDS_UNTIL_STOP_RETRY(elapsed) \
    (((elapsed) >= PRECONDITION_RETRY_US) ? 0U : \
     ((PRECONDITION_RETRY_US - (elapsed)) / 1000000U))

static int64_t ts_elapsed(int64_t now, int64_t old) {
    return now - old;
}

// ********************* state machine outline *********************
//
// IDLE                       Preconditioning is not requested.
// REQUESTED                  Owns an enabled session and prevents the car from cancelling it.
// +- CAR_START_DELAY         Waits briefly after READY before a persistent-mode relaunch.
// +- START_BURST (initial)   Sends the start command sequence.
// +- WAIT_STARTING           Waits for the car to begin starting.
// +- WAIT_STARTED            Waits for preconditioning to become fully active.
// +- ACTIVE                  Monitors active preconditioning in every mode.
// +- MANAGED                 Waits between attempts while a repeating mode remains enabled.
// STOPPING                   Owns an active stop request.
// +- STOP_BURST (initial)    Sends the stop command sequence.
// +- WAIT_STOPPED            Waits for the car to confirm that it stopped.
//
// Global hooks decode input and status frames independently of the active state.

enum {
    EV_TOGGLE,          // activation input fired (short press release / long press hold)
    EV_STATUS_IDLE,     // car reports preconditioning off/idle
    EV_STATUS_STARTING, // car reports preconditioning starting
    EV_STATUS_STARTED,  // car reports preconditioning fully running
    EV_CAR_READY,       // car power entered READY (0x038 edge)
    EV_CAR_NOT_READY,   // car power left READY (0x038 edge)
};

static const sm_state_t S_IDLE, S_REQUESTED, S_CAR_START_DELAY, S_START_BURST,
                        S_WAIT_STARTING, S_WAIT_STARTED, S_ACTIVE, S_MANAGED,
                        S_STOPPING, S_STOP_BURST, S_WAIT_STOPPED;

static sm_t precon_sm;

// ********************* state machine context *********************
// context is grouped by owner. the `requested`, `managed`, and `stopping`
// structs are engine-managed (.ctx on their states): they belong to those
// states and their children, and the engine zeroes them on entry so they can
// never carry stale values across episodes. `platform` and `button` belong to
// the global hooks and are machine-wide.

typedef enum {
    PRECON_STATUS_UNKNOWN = 0,
    PRECON_STATUS_IDLE,
    PRECON_STATUS_STARTING,
    PRECON_STATUS_STARTED,
} precon_status_t;

// why the current start attempt was launched. MANUAL must be zero: it is the
// entry argument plain sm_transition supplies
typedef enum {
    ATTEMPT_MANUAL = 0,  // activation button; shows the cluster countdown
    ATTEMPT_CAR_START,   // persistent-mode relaunch on a car-ready edge; shows the countdown
    ATTEMPT_PERIODIC,    // repeating-mode re-nudge; silent and one-shot
    ATTEMPT_BMU_RESTART, // status-only BMU restart; silent and not retried directly
} attempt_kind_t;

// owned by REQUESTED and its children; describes the most recent start
// attempt, so it is dormant while ACTIVE or MANAGED is the leaf
static struct {
    // why this attempt was launched; set on entry from the transition argument
    attempt_kind_t kind;
    // timestamp of the start of the most recent start burst, used for retry timing and the countdown display
    int64_t last_attempt_ts;
    // number of times we've re-sent the start burst within the current request
    uint8_t retries;
} requested;

// owned by MANAGED: scheduling for periodic start bursts while a repeating
// mode is enabled but preconditioning is inactive
static struct {
    // start of the current nudge interval
    int64_t nudge_base_ts;
} managed;

// owned by STOPPING and its children
static struct {
    // timestamp of the start of the most recent stop burst, used for retry timing and the retry display
    int64_t last_attempt_ts;
    // number of times we've re-sent the stop burst within the current stop
    uint8_t retries;
} stopping;

// process-lifetime platform info
static struct {
    // most recent recognized preconditioning status reported by the car
    precon_status_t precon_status;
    // is the car in READY? tracked from 0x038 edges; stays false on platforms
    // where that frame is unavailable
    bool car_in_ready;
} platform;

static bool precon_status_available(void) {
    return platform.precon_status != PRECON_STATUS_UNKNOWN;
}

// activation button edge tracking, owned by the global hooks
static struct {
    // is the button currently held? tracked for edge detection
    bool pressed;
    // timestamp of the press edge, for short/long press detection
    int64_t press_start_ts;
    // has the current hold already triggered? (long press mode fires once per hold)
    bool long_press_fired;
} button;

static QueueHandle_t battery_temperature_queue = NULL;
static QueueHandle_t car_power_queue = NULL;

// ********************* config snapshot *********************

// Config changes restart the firmware, so capture these values before the
// state machine is published to the tick and CAN tasks.
static struct {
    int8_t button_type;
    int8_t press_type;
    int8_t mode;
} precon_config;

// ********************* repeating-mode latch *********************

static bool repeating_mode(void) {
    return precon_config.mode == CONTINUOUS || precon_config.mode == PERSISTENT;
}

// RAM-only latch for continuous mode. Persistent mode, on the other hand,
// uses persistent_settings_(get|set)_precon_enabled, which has logic to
// asynchronously mirror the value to flash.
// This bool is owned by repeating_mode_enabled and set_repeating_enabled
// and shouldn't be accessed directly.
static bool continuous_enabled = false;

// is a repeating mode enabled? i.e. are we in continuous/persistent mode,
// with preconditioning toggled on?
static bool repeating_mode_enabled(void) {
    if (precon_config.mode == PERSISTENT) {
        return persistent_settings_get_precon_enabled();
    }
    return continuous_enabled;
}

static void set_repeating_enabled(bool enabled) {
    if (precon_config.mode == PERSISTENT) {
        persistent_settings_set_precon_enabled(enabled);
    } else {
        continuous_enabled = enabled;
    }
}

// ********************* CAN tx helpers *********************

// burst_tick counts up from 0 within the burst
static void send_precondition_start_msg(uint32_t burst_tick) {
    twai_message_t packet = {0};
    packet.identifier = 0x0C7U;
    packet.data_length_code = 8U;
    if (burst_tick < PRECONDITION_START_PHASE1_TICKS) {
        // send 0000004003000000 to 0x0C7
        packet.data[3] = 0x40U;
        packet.data[4] = 0x03U;
    } else {
        // send 000000E007000000 to 0x0C7
        packet.data[3] = 0xE0U;
        packet.data[4] = 0x07U;
    }
    // TODO(ejones): ensure that blocking for 1 tick is the right move here and elsewhere
    can_send(CAR_BUS, &packet, 1);
}

static void send_precondition_stop_msg(uint32_t burst_tick) {
    twai_message_t packet = {0};
    packet.identifier = 0x0C7U;
    packet.data_length_code = 8U;
    if (burst_tick >= PRECONDITION_STOP_PHASE1_TICKS) {
        // send 000000E007000000 to 0x0C7 (phase 1 sends all-zero data)
        packet.data[3] = 0xE0U;
        packet.data[4] = 0x07U;
    }
    can_send(CAR_BUS, &packet, 1);
}

// ********************* shared state helpers *********************

// 0x4E8/0x4CC countdown display while a start is in flight
// (shared by START_BURST, WAIT_STARTING, and WAIT_STARTED)
static fwd_result_t starting_display_fwd(sm_t *sm, twai_message_t *to_send, can_bus_t fwd_bus) {
    if (fwd_bus != CAR_BUS) {
        return FWD_PASSTHROUGH;
    }
    // periodic re-nudges and observed BMU restarts don't touch the cluster;
    // manual and car-start attempts show the countdown
    if (requested.kind == ATTEMPT_PERIODIC || requested.kind == ATTEMPT_BMU_RESTART) {
        return FWD_PASSTHROUGH;
    }
    // the display only runs until the car confirms preconditioning fully
    // started, which can happen mid-burst, before we route to ACTIVE
    if (platform.precon_status == PRECON_STATUS_STARTED) {
        return FWD_PASSTHROUGH;
    }
    if (to_send->identifier == 0x4E8U) {
        int64_t time_since_last_attempt = ts_elapsed(sm_now(sm), requested.last_attempt_ts);
        set_0x4e8_distance_flag(
            to_send,
            SECONDS_UNTIL_START(time_since_last_attempt),
            // display retry count in tenths digit
            precon_status_available() ? (requested.retries % 10U) : 0U,
            precon_status_available() ? (requested.retries == 0U ? DIST_UNIT_YD : DIST_UNIT_KM) : DIST_UNIT_M,
            // switch to the destination flag once the car confirms it's starting
            precon_status_available() ? (platform.precon_status == PRECON_STATUS_STARTING ? FLAG_DESTINATION : FLAG_BLUE_1) : FLAG_BLUE_4
        );
        return FWD_MODIFIED;
    }
    if (to_send->identifier == 0x4CCU) {
        to_send->data[0] = 0x02U;
        return FWD_MODIFIED;
    }
    return FWD_PASSTHROUGH;
}

// 0x4E8/0x4CC retry display while a stop is in flight
// (shared by STOP_BURST and WAIT_STOPPED)
static fwd_result_t stopping_display_fwd(sm_t *sm, twai_message_t *to_send, can_bus_t fwd_bus) {
    if (fwd_bus != CAR_BUS) {
        return FWD_PASSTHROUGH;
    }
    // only display when we can actually confirm/retry the stop, and hide it
    // once retries are exhausted (including the silent give-up stop)
    if (!precon_status_available() || stopping.retries >= PRECONDITION_MAX_RETRIES) {
        return FWD_PASSTHROUGH;
    }
    if (to_send->identifier == 0x4E8U) {
        int64_t time_since_last_attempt = ts_elapsed(sm_now(sm), stopping.last_attempt_ts);
        set_0x4e8_distance_flag(
            to_send,
            SECONDS_UNTIL_STOP_RETRY(time_since_last_attempt),
            // display retry count in tenths digit
            stopping.retries % 10U,
            stopping.retries == 0U ? DIST_UNIT_FT : DIST_UNIT_MI,
            FLAG_NONE
        );
        return FWD_MODIFIED;
    }
    if (to_send->identifier == 0x4CCU) {
        to_send->data[0] = 0x02U;
        return FWD_MODIFIED;
    }
    return FWD_PASSTHROUGH;
}

// a start attempt timed out: retry the burst, or give up
static void start_timeout(sm_t *sm) {
    if (requested.kind == ATTEMPT_PERIODIC || requested.kind == ATTEMPT_BMU_RESTART) {
        // Periodic bursts and BMU-observed restarts are one-shot: go back to
        // waiting in MANAGED without issuing an immediate retry.
        sm_transition(sm, &S_MANAGED);
    } else if (requested.retries < PRECONDITION_MAX_RETRIES) {
        // manual or car-start attempt timed out => continue with retry logic
        requested.retries++;
        sm_transition(sm, &S_START_BURST);
    } else if (repeating_mode_enabled()) {
        // retries exhausted in a repeating mode => go back to waiting in MANAGED
        sm_transition(sm, &S_MANAGED);
    } else {
        // retries exhausted in once mode => send silent stop request (then IDLE)
        sm_transition_arg(sm, &S_STOPPING, PRECONDITION_MAX_RETRIES);
    }
}

// ********************* IDLE *********************

static void idle_enter(sm_t *sm) {
    if (repeating_mode() && repeating_mode_enabled()) {
        // the WiCAN just booted and restored persistent mode from flash
        // => wait in MANAGED for car to boot
        sm_transition(sm, &S_MANAGED);
    }
}

static bool idle_event(sm_t *sm, sm_event_t ev) {
    switch (ev) {
        case EV_TOGGLE:
            sm_transition(sm, &S_REQUESTED);
            return true;
        case EV_CAR_READY:
            // we only stay in idle during once mode, so nothing to do
            // on car ready
            return true;
    }
    return false;
}

// ********************* REQUESTED (superstate) *********************

static void requested_enter(sm_t *sm) {
    requested.kind = (attempt_kind_t)sm_entry_arg(sm);
    if (repeating_mode()) {
        set_repeating_enabled(true);
    }
    if (requested.kind == ATTEMPT_BMU_RESTART) {
        requested.last_attempt_ts = sm_now(sm);
        // The status event already tells us how far the BMU got, so skip the
        // default start burst.
        sm_transition(sm, platform.precon_status == PRECON_STATUS_STARTED
                          ? &S_ACTIVE : &S_WAIT_STARTED);
    }
}

static bool requested_event(sm_t *sm, sm_event_t ev) {
    switch (ev) {
        case EV_TOGGLE:
            // debounce between start and stop
            if (sm_time_in_us(sm, &S_REQUESTED) > PRECONDITION_DEBOUNCE_US) {
                sm_transition(sm, &S_STOPPING);
            }
            return true;
        case EV_CAR_NOT_READY:
            // the car turning off already ended preconditioning: abandon the
            // session silently; there is nothing left to stop on the bus
            if (precon_config.mode == PERSISTENT && repeating_mode_enabled()) {
                // wait in MANAGED for the next car_ready rising edge.
                // when MANAGED is already the leaf this is a self-transition,
                // which usefully resets its stale start-burst ctx
                sm_transition(sm, &S_MANAGED);
            } else {
                // reset continuous/once mode state
                sm_transition(sm, &S_IDLE);
            }
            return true;
    }
    return false;
}

// the MITM runs for every child of REQUESTED, including ACTIVE and MANAGED:
// the head unit must not be able to cancel an enabled session
static fwd_result_t requested_fwd(sm_t *sm, twai_message_t *to_send, can_bus_t fwd_bus) {
    if (fwd_bus != CAR_BUS) {
        return FWD_PASSTHROUGH;
    }
    // block 0x0C7 so that the head unit doesn't turn off preconditioning on us
    // TODO(ejones): handle utility mode and test
    // (mitm 00 00 on bytes 4 and 5 to E0 07 (allows utility mode (byte 3, 80) to go through))
    if (to_send->identifier == 0x0C7U) {
        return FWD_BLOCK;
    }
    // MITM 0x4ED while preconditioning is requested
    if (to_send->identifier == 0x4EDU) {
        to_send->data[5] = 0x10U;
        to_send->data[6] = 0xA0U;
        to_send->data[7] = 0x00U;
        return FWD_MODIFIED;
    }
    return FWD_PASSTHROUGH;
}

static void requested_exit(sm_t *sm) {
    if (repeating_mode()) {
        set_repeating_enabled(false);
    }
}

// ********************* REQUESTED / CAR_START_DELAY *********************

static void car_start_delay_tick(sm_t *sm) {
    if (sm_time_in_us(sm, &S_CAR_START_DELAY) >= PRECONDITION_CAR_START_DELAY_US) {
        // Re-enter REQUESTED so the attempt context starts fresh and records
        // that this is the persistent-mode relaunch rather than a manual try.
        sm_transition_arg(sm, &S_REQUESTED, ATTEMPT_CAR_START);
    }
}

// ********************* REQUESTED / START_BURST *********************

// retry timers and the countdown display measure from the moment the burst began
static void start_burst_enter(sm_t *sm) {
    requested.last_attempt_ts = sm_now(sm);
}

static void start_burst_tick(sm_t *sm) {
    uint32_t t = sm_ticks_in_state(sm);
    send_precondition_start_msg(t);
    // Status events deliberately do not cut the burst short. After all start
    // messages are sent, route using the car's latest reported status.
    if (t + 1U >= PRECONDITION_START_TICKS) {
        switch (platform.precon_status) {
            case PRECON_STATUS_STARTED:
                sm_transition(sm, &S_ACTIVE);
                break;
            case PRECON_STATUS_STARTING:
                sm_transition(sm, &S_WAIT_STARTED);
                break;
            default:
                sm_transition(sm, &S_WAIT_STARTING);
                break;
        }
    }
}

// ********************* REQUESTED / WAIT_STARTING *********************

static void wait_starting_tick(sm_t *sm) {
    int64_t time_since_last_attempt = ts_elapsed(sm_now(sm), requested.last_attempt_ts);
    if (!precon_status_available()) {
        // without status frames we can't confirm or retry anything; after the
        // timeout, assume it worked so the countdown display goes away
        if (time_since_last_attempt > PRECONDITION_STARTED_TIMEOUT_US) {
            sm_transition(sm, &S_ACTIVE);
        }
        return;
    }
    if (time_since_last_attempt > PRECONDITION_RETRY_US) {
        start_timeout(sm);
    }
}

static bool wait_starting_event(sm_t *sm, sm_event_t ev) {
    switch (ev) {
        case EV_STATUS_STARTING:
            sm_transition(sm, &S_WAIT_STARTED);
            return true;
        case EV_STATUS_STARTED:
            sm_transition(sm, &S_ACTIVE);
            return true;
        case EV_STATUS_IDLE:
            // expected; BMU usually takes a little while to register the start burst
            return true;
    }
    return false;
}

// ********************* REQUESTED / WAIT_STARTED *********************

static void wait_started_tick(sm_t *sm) {
    // the car said "starting" but hasn't reached fully started
    // (i.e. we got 2AD 05 but not 15 after a long time)
    int64_t time_since_last_attempt = ts_elapsed(sm_now(sm), requested.last_attempt_ts);
    if (time_since_last_attempt > PRECONDITION_STARTED_TIMEOUT_US) {
        start_timeout(sm);
    }
}

static bool wait_started_event(sm_t *sm, sm_event_t ev) {
    switch (ev) {
        case EV_STATUS_STARTED:
            sm_transition(sm, &S_ACTIVE);
            return true;
        case EV_STATUS_STARTING:
            // still starting; keep waiting
            return true;
        case EV_STATUS_IDLE:
            // a bit odd; we previously got "starting", but now the BMU is showing "idle".
            // perhaps the SoC/temp conditions are no longer met?
            // let's just stick with 70s retries for now.
            return true;
    }
    return false;
}

// ********************* REQUESTED / ACTIVE *********************
// Preconditioning is currently enabled.

static bool active_event(sm_t *sm, sm_event_t ev) {
    switch (ev) {
        case EV_STATUS_STARTED:
            // still started; all is well.
            return true;
        case EV_STATUS_STARTING:
            // preconditioning was previously fully active, but now it's only showing as starting.
            if (repeating_mode()) {
                // The BMU is presumably restarting on its own, so monitor it 
                // with a fresh silent attempt context.
                sm_transition_arg(sm, &S_REQUESTED, ATTEMPT_BMU_RESTART);
            } else {
                // Once mode: keep retrying with original retry budget.
                requested.last_attempt_ts = sm_now(sm);
                sm_transition(sm, &S_WAIT_STARTED);
            }
            return true;
        case EV_STATUS_IDLE:
            // preconditioning was previously fully active, but now it's showing as off.
            // the battery has probably reached the target temp or fallen below the
            // SoC threshold. (TODO(ejones): get a CAN recording of these scenarios)
            if (repeating_mode_enabled()) {
                // Keep repeating-mode on and wait before asking the
                // BMU to start again.
                sm_transition(sm, &S_MANAGED);
            } else {
                // Once mode actively stops. This, together with the stopping
                // MITM, prevents preconditioning from restarting after the
                // battery falls back below the target temperature.
                sm_transition(sm, &S_STOPPING);
            }
            return true;
    }
    return false;
}

// ********************* REQUESTED / MANAGED *********************
// Continuous/persistent modes only. In this state, the user is
// actively requesting preconditioning, but it is not currently started
// due to restrictions on SoC/temp/etc.

static void managed_enter(sm_t *sm) {
    managed.nudge_base_ts = sm_now(sm);
}

static void managed_tick(sm_t *sm) {
    if (platform.car_in_ready
            && ts_elapsed(sm_now(sm), managed.nudge_base_ts) > REPEATING_MODE_RETRY_INTERVAL_US) {
        // targets the parent, so REQUESTED exits and re-enters: fresh attempt
        // ctx, kind set from the entry argument, descend into START_BURST
        sm_transition_arg(sm, &S_REQUESTED, ATTEMPT_PERIODIC);
    }
}

static bool managed_event(sm_t *sm, sm_event_t ev) {
    switch (ev) {
        case EV_STATUS_IDLE:
            return true;
        case EV_STATUS_STARTING:
        case EV_STATUS_STARTED:
            // The BMU has restarted preconditioning on its own:
            // start a new silent attempt that skips the start burst.
            sm_transition_arg(sm, &S_REQUESTED, ATTEMPT_BMU_RESTART);
            return true;
        case EV_CAR_READY:
            // persistent mode relaunches after the car has had time to finish
            // coming up in READY
            if (precon_config.mode == PERSISTENT) {
                sm_transition(sm, &S_CAR_START_DELAY);
            }
            return true;
    }
    // EV_TOGGLE and EV_CAR_NOT_READY bubble to REQUESTED
    return false;
}

// ********************* STOPPING (superstate) *********************

// the entry argument carries the initial retry count: 0 (the default) for a
// normal stop, PRECONDITION_MAX_RETRIES for a single silent burst (no display,
// no retries), which is how a failed start gives up
static void stopping_enter(sm_t *sm) {
    stopping.retries = (uint8_t)sm_entry_arg(sm);
}

static bool stopping_event(sm_t *sm, sm_event_t ev) {
    switch (ev) {
        case EV_TOGGLE:
            // activation while stopping restarts preconditioning
            sm_transition(sm, &S_REQUESTED);
            return true;
        case EV_CAR_NOT_READY:
            // the car turning off ended preconditioning anyway; abandon the stop
            sm_transition(sm, &S_IDLE);
            return true;
    }
    return false;
}

// ********************* STOPPING / STOP_BURST *********************

// retry timers and the retry display measure from the moment the burst began
static void stop_burst_enter(sm_t *sm) {
    stopping.last_attempt_ts = sm_now(sm);
}

static void stop_burst_tick(sm_t *sm) {
    uint32_t t = sm_ticks_in_state(sm);
    send_precondition_stop_msg(t);
    if (t + 1U >= PRECONDITION_STOP_TICKS) {
        // without status frames there's no confirmation or retry to wait for
        sm_transition(sm, precon_status_available() ? &S_WAIT_STOPPED : &S_IDLE);
    }
}

static bool stop_burst_event(sm_t *sm, sm_event_t ev) {
    if (ev == EV_STATUS_IDLE) {
        // deliberately not treated as confirmation: the stop only counts as
        // confirmed once the full burst has been sent
        return true;
    }
    return false;
}

// ********************* STOPPING / WAIT_STOPPED *********************

static void wait_stopped_tick(sm_t *sm) {
    int64_t time_since_last_attempt = ts_elapsed(sm_now(sm), stopping.last_attempt_ts);
    if (time_since_last_attempt > PRECONDITION_RETRY_US) {
        if (stopping.retries < PRECONDITION_MAX_RETRIES) {
            stopping.retries++;
            sm_transition(sm, &S_STOP_BURST);
        } else {
            // give up; the retry display is already hidden at max retries
            sm_transition(sm, &S_IDLE);
        }
    }
}

static bool wait_stopped_event(sm_t *sm, sm_event_t ev) {
    if (ev == EV_STATUS_IDLE) {
        sm_transition(sm, &S_IDLE);
        return true;
    }
    return false;
}

// ********************* state table *********************

static const sm_state_t S_IDLE = {
    .name = "idle",
    .enter = idle_enter,
    .event = idle_event,
};

static const sm_state_t S_REQUESTED = {
    .name = "requested",
    .initial = &S_START_BURST,
    .ctx = &requested,
    .ctx_size = sizeof(requested),
    .enter = requested_enter,
    .event = requested_event,
    .fwd = requested_fwd,
    .exit = requested_exit,
};

static const sm_state_t S_CAR_START_DELAY = {
    .name = "car-start-delay",
    .parent = &S_REQUESTED,
    .tick = car_start_delay_tick,
};

static const sm_state_t S_START_BURST = {
    .name = "start-burst",
    .parent = &S_REQUESTED,
    .enter = start_burst_enter,
    .tick = start_burst_tick,
    .fwd = starting_display_fwd,
};

static const sm_state_t S_WAIT_STARTING = {
    .name = "wait-starting",
    .parent = &S_REQUESTED,
    .tick = wait_starting_tick,
    .event = wait_starting_event,
    .fwd = starting_display_fwd,
};

static const sm_state_t S_WAIT_STARTED = {
    .name = "wait-started",
    .parent = &S_REQUESTED,
    .tick = wait_started_tick,
    .event = wait_started_event,
    .fwd = starting_display_fwd,
};

static const sm_state_t S_ACTIVE = {
    .name = "active",
    .parent = &S_REQUESTED,
    .event = active_event,
};

static const sm_state_t S_MANAGED = {
    .name = "managed",
    .parent = &S_REQUESTED,
    .ctx = &managed,
    .ctx_size = sizeof(managed),
    .enter = managed_enter,
    .tick = managed_tick,
    .event = managed_event,
};

static const sm_state_t S_STOPPING = {
    .name = "stopping",
    .initial = &S_STOP_BURST,
    .ctx = &stopping,
    .ctx_size = sizeof(stopping),
    .enter = stopping_enter,
    .event = stopping_event,
};

static const sm_state_t S_STOP_BURST = {
    .name = "stop-burst",
    .parent = &S_STOPPING,
    .enter = stop_burst_enter,
    .tick = stop_burst_tick,
    .event = stop_burst_event,
    .fwd = stopping_display_fwd,
};

static const sm_state_t S_WAIT_STOPPED = {
    .name = "wait-stopped",
    .parent = &S_STOPPING,
    .tick = wait_stopped_tick,
    .event = wait_stopped_event,
    .fwd = stopping_display_fwd,
};

// ********************* global hooks *********************

static void precondition_global_tick(sm_t *sm) {
    // long press mode: trigger once when the hold crosses the threshold, without
    // waiting for the release frame. state only becomes pressed via the rx hook,
    // so this does nothing when the activation button is disabled
    if (button.pressed && !button.long_press_fired
            && precon_config.press_type == PRESS_LONG
            && ts_elapsed(sm_now(sm), button.press_start_ts) >= PRECONDITION_LONG_PRESS_US) {
        button.long_press_fired = true;
        sm_send_event(sm, EV_TOGGLE);
    }
}

static void precondition_global_rx(sm_t *sm, const twai_message_t *to_push, can_bus_t rx_bus) {
    // 0x038 power status: the low nibble of byte 0 reads 0x04 while the car is
    // in READY. only act on edges
    if (IS_POWER_STATUS_FRAME(to_push->identifier)
            && rx_bus == CAR_BUS
            && to_push->data_length_code >= 1U) {
        bool ready = POWER_STATUS_READY(to_push->data[0]);

        // Publish every frame, not just edges: a consumer needs to tell "in
        // READY right now" from "READY was the last thing seen before the bus
        // went quiet", and only a per-frame timestamp does that.
        precondition_power_t power = {
            .ready = ready,
            .charging = POWER_STATUS_CHARGING(to_push->data[0]),
            .raw = to_push->data[0],
            .updated_at_us = sm_now(sm),
        };
        xQueueOverwrite(car_power_queue, &power);

        if (ready != platform.car_in_ready) {
            platform.car_in_ready = ready;
            ESP_LOGI(TAG, "car power: %s", ready ? "ready" : "off");
            sm_send_event(sm, ready ? EV_CAR_READY : EV_CAR_NOT_READY);
        }
    }

    // 0x2AD/0x0A82AA03 status frame: second byte indicates precondition state
    //   Ioniq 5/6: 0x01 = off/idle, 0x05 = starting, 0x15 = fully running
    //   EV6: 0x41 = off/idle, 0x45 = starting, 0x55 = fully running
    // only trust status frames coming from the car itself; a same-ID frame on
    // the head unit bus must not drive the state machine
    if (IS_STATUS_FRAME(to_push->identifier) && rx_bus == CAR_BUS) {
        uint8_t status = to_push->data[1];
        // precon_status => last status we've seen
        // EV_STATUS_... => instantaneous triggered by getting a status frame
        if (STATUS_STARTED(status)) {
            platform.precon_status = PRECON_STATUS_STARTED;
            sm_send_event(sm, EV_STATUS_STARTED);
        } else if (STATUS_STARTING(status)) {
            platform.precon_status = PRECON_STATUS_STARTING;
            sm_send_event(sm, EV_STATUS_STARTING);
        } else if (STATUS_IDLE(status)) {
            platform.precon_status = PRECON_STATUS_IDLE;
            sm_send_event(sm, EV_STATUS_IDLE);
        }
    }

    if (IS_BATTERY_TEMPERATURE_FRAME(to_push->identifier)
            && rx_bus == CAR_BUS
            && to_push->data_length_code >= BATTERY_TEMPERATURE_DATA_LENGTH) {
        precondition_temperature_t temperature = {
            .min_c = to_push->data[BATTERY_TEMPERATURE_MIN_INDEX],
            .max_c = to_push->data[BATTERY_TEMPERATURE_MAX_INDEX],
            .updated_at_us = sm_now(sm),
        };

        xQueueOverwrite(battery_temperature_queue, &temperature);
    }

    int8_t precon_button_type = precon_config.button_type;
    if (precon_button_type == BUTTON_DISABLED) {
        // activation button disabled in config; don't listen for any button press
        return;
    }
    if (precon_button_type < 0 || precon_button_type >= NUM_PRECON_BUTTONS) {
        ESP_LOGE(TAG, "Invalid precondition button type: %d", precon_button_type);
        return;
    }
    // track activation button press/release edges. short press mode triggers on
    // the release edge if the hold stayed under the threshold; long press mode
    // triggers from the tick hook once the hold crosses the threshold
    const message_payload_t *activation = &activation_messages[precon_button_type];
    if (activation_is_press(activation, to_push)) {
        if (!button.pressed) {
            button.press_start_ts = sm_now(sm);
            button.long_press_fired = false;
        }
        button.pressed = true;
    } else if (activation_is_release(activation, to_push)) {
        if (button.pressed
                && precon_config.press_type == PRESS_SHORT
                && ts_elapsed(sm_now(sm), button.press_start_ts) < PRECONDITION_LONG_PRESS_US) {
            sm_send_event(sm, EV_TOGGLE);
        }
        button.pressed = false;
    }
}

static const sm_hooks_t precondition_global_hooks = {
    .tick = precondition_global_tick,
    .rx = precondition_global_rx,
};

// ********************* public API *********************

void precondition_init(void) {
    precon_config.button_type = config_server_precon_button();
    precon_config.press_type = config_server_precon_press();
    precon_config.mode = config_server_precon_mode();

    if (precon_config.mode == PERSISTENT) {
        // Currently our only persistent setting is whether persistent
        // mode was enabled. If that changes, we should make this init
        // unconditional.
        persistent_settings_init();
    }

    battery_temperature_queue = xQueueCreate(1, sizeof(precondition_temperature_t));
    configASSERT(battery_temperature_queue != NULL);
    car_power_queue = xQueueCreate(1, sizeof(precondition_power_t));
    configASSERT(car_power_queue != NULL);
    sm_init(&precon_sm, "precondition", &S_IDLE, &precondition_global_hooks);
}

// called every 40ms
void precondition_tick(void) {
    sm_tick(&precon_sm);
}

void precondition_can_rx_hook(twai_message_t *to_push, can_bus_t rx_bus) {
    sm_rx(&precon_sm, to_push, rx_bus);
}

// Decide whether to block, modify, or passthrough a message for preconditioning.
// Modifies packet data in-place when returning FWD_MODIFIED.
// On single-bus builds only FWD_MODIFIED has an effect (FWD_BLOCK can't pull a
// frame that's already on the wire); on multi-bus builds the caller bridges,
// so FWD_BLOCK and FWD_PASSTHROUGH matter too. fwd_bus is the destination bus.
fwd_result_t precondition_fwd_hook(twai_message_t *to_send, can_bus_t fwd_bus) {
    return sm_fwd(&precon_sm, to_send, fwd_bus);
}

bool precondition_get_battery_temperature(precondition_temperature_t *out) {
    if (out == NULL || battery_temperature_queue == NULL) {
        return false;
    }

    return xQueuePeek(battery_temperature_queue, out, 0) == pdTRUE;
}

bool precondition_get_car_power(precondition_power_t *out) {
    if (out == NULL || car_power_queue == NULL) {
        return false;
    }

    return xQueuePeek(car_power_queue, out, 0) == pdTRUE;
}
