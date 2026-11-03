#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "can.h"
#include "climate.h"
#include "hw_config.h"

#define TAG "climate"

#if HW_HAS_MCP2515

// ********************* what the head unit sends *********************
//
// Reverse-engineered from a real Bluelink press, logged on M-CAN while the
// head unit's second bus (I-CAN) was captured in parallel:
//
//   WiCAN_MITM/Dual_M-CAN_MITM_I-CAN_parallel_climate_start_stop.csv
//
// A remote climate start is:
//
//   t+0.000s  0x4F1  00 D0 00 00 00 00 00 00   wake/session bit, x3
//   t+1.036s  0x4A2  00 00 0F 03 00 00 00 00   request (climate), x3
//   t+1.160s  0x4A2  00 00 FF 03 FF FF 00 00   apply, x3
//             0x0C7  00 00 00 01 00 00 00 00   climate opcode, x3, paired
//   t+1.272s  0x0C7  00 00 00 00 00 00 00 00   trailer, x3
//   ...       0x4F1  00 D0 ...                held for ~8 s at 1 Hz
//   t+8.100s  0x4F1  00 C0 ...                session closed, x3
//
// The car answers ~0.6 s after the apply burst: 0x2AD moves to its running
// state and 0x380 becomes 3A 85 00 06 06 06 06 01 (DATC up), then 0x037
// reports the HV system engaged. The stop direction carries no 0x4A2 at all,
// just 0x4F1 00 D0 and the car dropping out of the running state.
//
// 0x0C7 byte 3 = 0x01 is optional: one press sent it, two others did not and
// still started climate. It is replayed here because the dual-bus capture has
// it, and its effect (if any) is unknown.
//
// The head unit's second bus carries the session heartbeat 0x39B
// (`50 06 00 XX 00 03 10 00`, XX counting 00, 04 .. 3C at 2 Hz, byte 0 = 0x50
// while running, 0x40 as it closes). It is not transmitted on M-CAN. This
// build sends it on M-CAN anyway: if anything on the car side listens for it
// through the gateway, this is where it would show up, and if nothing does it
// is six harmless bytes every 200 ms for eight seconds.
//
// None of these frames carries authentication, a counter, or a seed/key
// exchange: 0x4A2 bytes 0-1 are 0x00 in every payload ever captured, the
// payloads are byte-identical across model years and head units, and no 0x7xx
// diagnostic traffic appears in any remote-climate capture. The three-frame
// repetition is the same latch the 0x0C7 preconditioning commands use.
//
// ********************* what the car answers *********************
//
// The captures also show which frames move when the car accepts the request.
// The Status tab reports the last one it sees, because none of them is
// documented and their exact meaning is still being worked out:
//
//   0x037  byte 0          vehicle power state. 0x00 while off, 0x22/0x23 once
//                          the HV system engages, which follows a successful
//                          remote climate start by about 0.1 s.
//   0x2AD  byte 1          BMS precondition state; 00 02 -> 04 01 when the
//                          car starts the remote request.
//   0x380  bytes 3..6      DATC setpoints (car bus). 00 00 00 00 while the
//                          climate unit is asleep, 06 06 06 06 once it is up.
//   0x4DD  byte 2          state bit. 0xFE -> 0x00 at the 0x4A2 apply.
//   0x4EB  byte 4 bit 7    head unit's active flag.
//   0x4A2  bytes 2..3      the request itself, seen on either bus.
//   0x0C7  bytes 3..4      the climate opcode and its trailer, echoed back.
//   0x4F1  byte 1          the wake bit, on while a remote session is open.
//   0x39B  byte 0          I-CAN session heartbeat (not received on M-CAN).

#define CLIMATE_CAR_BUS CAN_BUS_0

#define CLIMATE_WAKE_FRAME_ID 0x4F1U
#define CLIMATE_REQUEST_FRAME_ID 0x4A2U
#define CLIMATE_OPCODE_FRAME_ID 0x0C7U
#define CLIMATE_TMU_FRAME_ID 0x39BU

// Every command is repeated three times, matching the head unit and the 0x0C7
// burst helper in precondition.c.
#define CLIMATE_BURST_TICKS 3U

// The head unit holds the wake bit for ~1.04 s before it sends 0x4A2.
#define CLIMATE_WAKE_LEAD_TICKS 26U

// 0x39B and 0x4DD run at 5 Hz during the session, 0x4F1 at 1 Hz.
#define CLIMATE_SESSION_PERIOD_TICKS 5U
#define CLIMATE_KEEPALIVE_TICKS 25U

// The head unit keeps the session open for about 8 s in total (wake burst to
// the closing 0x4F1 00 C0 burst); everything after the trailer is this phase.
#define CLIMATE_SESSION_TICKS 170U

#define CLIMATE_TRAILER_TICK \
    (CLIMATE_WAKE_LEAD_TICKS + 2U * CLIMATE_BURST_TICKS)
#define CLIMATE_SESSION_TICK (CLIMATE_TRAILER_TICK + CLIMATE_BURST_TICKS)
#define CLIMATE_SEQUENCE_TICKS (CLIMATE_SESSION_TICK + CLIMATE_SESSION_TICKS)
#define CLIMATE_CLEAR_TICKS (CLIMATE_SEQUENCE_TICKS + CLIMATE_BURST_TICKS)

// ********************* sequence storage *********************

typedef struct {
    // Set by the web UI handler, cleared by the tick when the sequence starts.
    // Blocking keys off this too, so the request takes effect immediately.
    _Atomic bool requested;
    _Atomic bool active;
    // Ticks since the sequence started; owned by the precondition task.
    _Atomic uint32_t tick;
    // Free-running 40 ms counter, so a received frame can be timestamped
    // against the start of the sequence whichever task sees it.
    _Atomic uint32_t now_tick;
    _Atomic uint32_t start_tick;
    _Atomic bool started;
    // Last frame that looked like the car answering, packed by pack_event().
    _Atomic uint32_t event;
    _Atomic uint32_t event_tick;
    _Atomic bool event_valid;
    // 0x39B's counter byte, advanced only by the tick.
    uint8_t session_counter;
} climate_state_t;

static climate_state_t climate;

// Event word: id in bits 0..14, above that the two payload bytes worth
// reporting, then which bus it arrived on, so one atomic load is a complete
// reading (the id is 11 bits, so bit 15 is free).
#define CLIMATE_EVENT_HU 0x8000U

// ********************* payloads *********************

static const uint8_t wake_data[8] = {0x00U, 0xD0U, 0x00U, 0x00U,
                                     0x00U, 0x00U, 0x00U, 0x00U};
static const uint8_t idle_data[8] = {0x00U, 0xC0U, 0x00U, 0x00U,
                                     0x00U, 0x00U, 0x00U, 0x00U};
static const uint8_t request_data[8] = {0x00U, 0x00U, 0x0FU, 0x03U,
                                        0x00U, 0x00U, 0x00U, 0x00U};
static const uint8_t apply_data[8] = {0x00U, 0x00U, 0xFFU, 0x03U,
                                      0xFFU, 0xFFU, 0x00U, 0x00U};
static const uint8_t opcode_data[8] = {0x00U, 0x00U, 0x00U, 0x01U,
                                       0x00U, 0x00U, 0x00U, 0x00U};
static const uint8_t trailer_data[8] = {0x00U, 0x00U, 0x00U, 0x00U,
                                        0x00U, 0x00U, 0x00U, 0x00U};
// 0x4DD is 5 Hz on this car and reads 08 in byte 4 while it is awake; only
// byte 2 changes when the session opens.
static const uint8_t state_data[8] = {0x00U, 0x00U, 0x00U, 0x00U,
                                      0x08U, 0x00U, 0x00U, 0x00U};

static void send_climate_frame(uint32_t identifier, const uint8_t data[8]) {
    twai_message_t packet = {0};
    packet.identifier = identifier;
    packet.data_length_code = 8U;
    memcpy(packet.data, data, 8U);
    can_send(CLIMATE_CAR_BUS, &packet, 1);
}

static void send_wake_frame(void) {
    send_climate_frame(CLIMATE_WAKE_FRAME_ID, wake_data);
}

static void send_idle_frame(void) {
    send_climate_frame(CLIMATE_WAKE_FRAME_ID, idle_data);
}

static void send_request_frame(void) {
    send_climate_frame(CLIMATE_REQUEST_FRAME_ID, request_data);
}

static void send_apply_frames(void) {
    send_climate_frame(CLIMATE_REQUEST_FRAME_ID, apply_data);
    send_climate_frame(CLIMATE_OPCODE_FRAME_ID, opcode_data);
}

static void send_trailer_frame(void) {
    send_climate_frame(CLIMATE_OPCODE_FRAME_ID, trailer_data);
}

// The I-CAN session heartbeat, sent here on the car bus as an experiment.
static void send_tmu_frame(bool running) {
    uint8_t data[8] = {(uint8_t)(running ? 0x50U : 0x00U), 0x06U, 0x00U,
                       climate.session_counter, 0x00U, 0x03U, 0x10U, 0x00U};
    climate.session_counter = (uint8_t)((climate.session_counter + 4U) & 0x3FU);
    send_climate_frame(CLIMATE_TMU_FRAME_ID, data);
}

// Keeps the session alive: the heartbeat and the state frame at 5 Hz, the
// wake bit at 1 Hz, on their own offsets inside a 200 ms period.
static void send_session_frames(uint32_t tick, bool running) {
    const uint32_t phase = tick % CLIMATE_SESSION_PERIOD_TICKS;
    if (phase == 3U) {
        send_tmu_frame(running);
    } else if (phase == 4U) {
        send_climate_frame(0x4DDU, state_data);
    }
    if ((tick % CLIMATE_KEEPALIVE_TICKS) == 11U) {
        send_wake_frame();
    }
}

// ********************* response log *********************

// Only a couple of bytes per frame say anything about climate; the rest is a
// counter, a clock or a checksum. These are the ones seen moving in the
// captures (see the header comment).
typedef struct {
    uint32_t id;
    uint8_t first;
    uint8_t second;
} climate_watch_t;

static const climate_watch_t climate_watch[] = {
    {CLIMATE_WAKE_FRAME_ID, 1U, 0U},      // wake bit, stay-alive byte
    {CLIMATE_REQUEST_FRAME_ID, 2U, 3U},   // request mask, command byte
    {CLIMATE_OPCODE_FRAME_ID, 3U, 4U},    // opcode, its trailer
    {CLIMATE_TMU_FRAME_ID, 0U, 3U},       // I-CAN session heartbeat
    {0x037U, 0U, 1U},                     // vehicle power state
    {0x4DDU, 2U, 4U},                     // state byte, mode
    {0x4EBU, 4U, 2U},                     // active flag, minutes
    {0x380U, 3U, 4U},                     // DATC setpoints
    {0x2ADU, 1U, 0U},                     // BMS precondition state
};

static uint32_t pack_event(uint32_t id, uint8_t first, uint8_t second,
                           can_bus_t bus) {
    return id | ((uint32_t)first << 16) | ((uint32_t)second << 24)
           | (bus == CLIMATE_CAR_BUS ? 0U : CLIMATE_EVENT_HU);
}

static void log_event(uint32_t event) {
    // these frames repeat every few hundred milliseconds; only edges matter
    if (event == atomic_load_explicit(&climate.event, memory_order_relaxed)) {
        return;
    }
    atomic_store_explicit(&climate.event, event, memory_order_relaxed);
    atomic_store_explicit(&climate.event_tick,
                          atomic_load_explicit(&climate.now_tick, memory_order_relaxed),
                          memory_order_relaxed);
    atomic_store_explicit(&climate.event_valid, true, memory_order_relaxed);

    const char *bus = (event & CLIMATE_EVENT_HU) != 0U ? "head-unit" : "car";
    const unsigned id = (unsigned)(event & 0x7FFU);
    const unsigned first = (unsigned)((event >> 16) & 0xFFU);
    const unsigned second = (unsigned)((event >> 24) & 0xFFU);

    // say how far into the replay the response arrived, which is the only
    // thing that makes these lines useful later
    if (atomic_load_explicit(&climate.started, memory_order_relaxed)) {
        const uint32_t start = atomic_load_explicit(&climate.start_tick, memory_order_relaxed);
        const uint32_t now = atomic_load_explicit(&climate.now_tick, memory_order_relaxed);
        const int32_t delta = (int32_t)(now - start);
        const uint32_t since = delta > 0 ? (uint32_t)delta * 4U / 10U : 0U;
        ESP_LOGI(TAG, "response 0x%03X %02X %02X on the %s bus at +%u.%u s", id,
                 first, second, bus, (unsigned)(since / 10U), (unsigned)(since % 10U));
    } else {
        ESP_LOGI(TAG, "response 0x%03X %02X %02X on the %s bus", id, first, second,
                 bus);
    }
}

void climate_rx_hook(const twai_message_t *msg, can_bus_t rx_bus) {
    if (msg == NULL || msg->data_length_code < 8U) {
        return;
    }
    for (size_t i = 0; i < sizeof(climate_watch) / sizeof(climate_watch[0]); i++) {
        if (msg->identifier != climate_watch[i].id) {
            continue;
        }
        const uint8_t first = msg->data[climate_watch[i].first];
        const uint8_t second = msg->data[climate_watch[i].second];
        // skip the idle states of the two frames that also carry a trailer
        if (msg->identifier == CLIMATE_OPCODE_FRAME_ID && first == 0U
                && second == 0U) {
            return;
        }
        if (msg->identifier == CLIMATE_WAKE_FRAME_ID && (first & 0x10U) == 0U) {
            return;
        }
        log_event(pack_event(msg->identifier, first, second, rx_bus));
        return;
    }
}

// ********************* public API *********************

static bool climate_blocking(void) {
    return atomic_load_explicit(&climate.requested, memory_order_relaxed)
           || atomic_load_explicit(&climate.active, memory_order_relaxed);
}

void climate_tick(void) {
    const uint32_t now = atomic_fetch_add_explicit(
        &climate.now_tick, 1U, memory_order_relaxed);

    if (!atomic_load_explicit(&climate.active, memory_order_relaxed)) {
        if (!atomic_exchange_explicit(&climate.requested, false,
                                      memory_order_relaxed)) {
            return;
        }
        atomic_store_explicit(&climate.active, true, memory_order_relaxed);
        atomic_store_explicit(&climate.tick, 0U, memory_order_relaxed);
        atomic_store_explicit(&climate.start_tick, now, memory_order_relaxed);
        atomic_store_explicit(&climate.started, true, memory_order_relaxed);
        climate.session_counter = 0U;
        ESP_LOGI(TAG, "remote climate start: replaying head unit sequence");
    }

    const uint32_t tick = atomic_fetch_add_explicit(
        &climate.tick, 1U, memory_order_relaxed);
    if (tick < CLIMATE_BURST_TICKS) {
        send_wake_frame();
    } else if (tick < CLIMATE_WAKE_LEAD_TICKS) {
        send_session_frames(tick, false);
    } else if (tick < CLIMATE_WAKE_LEAD_TICKS + CLIMATE_BURST_TICKS) {
        send_request_frame();
        send_session_frames(tick, false);
    } else if (tick < CLIMATE_WAKE_LEAD_TICKS + 2U * CLIMATE_BURST_TICKS) {
        send_apply_frames();
        send_session_frames(tick, true);
    } else if (tick < CLIMATE_SESSION_TICK) {
        send_trailer_frame();
        send_session_frames(tick, true);
    } else if (tick < CLIMATE_SEQUENCE_TICKS) {
        send_session_frames(tick, true);
    } else if (tick < CLIMATE_CLEAR_TICKS) {
        // close the session the way the head unit does, so the car does not
        // think a remote request is still open
        send_idle_frame();
    } else {
        atomic_store_explicit(&climate.active, false, memory_order_relaxed);
        ESP_LOGI(TAG, "remote climate start: sequence finished");
    }
}

fwd_result_t climate_fwd_hook(twai_message_t *msg, can_bus_t fwd_bus) {
    if (fwd_bus != CLIMATE_CAR_BUS || !climate_blocking()) {
        return FWD_PASSTHROUGH;
    }
    if (msg->identifier == CLIMATE_REQUEST_FRAME_ID
            || msg->identifier == CLIMATE_WAKE_FRAME_ID) {
        // The head unit starts the same remote climate on its own as soon as
        // the request reaches it; let only our replay through to the car.
        return FWD_BLOCK;
    }
    return FWD_PASSTHROUGH;
}

esp_err_t climate_request_start(void) {
    // A request that is still waiting for the tick to pick it up counts as
    // busy too: the UI disables its button on climate_is_active(), which is
    // exactly this condition.
    if (climate_blocking()) {
        return ESP_ERR_INVALID_STATE;
    }
    // Block the head unit's copies from here, before the precondition task
    // gets a chance to start the sequence.
    atomic_store_explicit(&climate.requested, true, memory_order_relaxed);
    return ESP_OK;
}

bool climate_is_active(void) {
    return climate_blocking();
}

bool climate_get_status(char *out, size_t len) {
    if (out == NULL || len == 0U) {
        return false;
    }

    const bool active = atomic_load_explicit(&climate.active, memory_order_relaxed);
    const bool queued = !active
        && atomic_load_explicit(&climate.requested, memory_order_relaxed);
    const bool started = atomic_load_explicit(&climate.started, memory_order_relaxed);
    const uint32_t start_tick = atomic_load_explicit(&climate.start_tick, memory_order_relaxed);
    const uint32_t tick = atomic_load_explicit(&climate.tick, memory_order_relaxed);

    // one tick is 40 ms, so tick * 4 / 10 is tenths of a second. The buffer
    // is sized for the worst case the compiler assumes (ten digits per %u),
    // not for the 8.3 s the sequence actually takes.
    char phase[40];
    if (active) {
        const uint32_t elapsed = tick * 4U / 10U;
        const uint32_t total = (uint32_t)CLIMATE_CLEAR_TICKS * 4U / 10U;
        snprintf(phase, sizeof(phase), "replaying %u.%u/%u.%u s",
                 (unsigned)(elapsed / 10U), (unsigned)(elapsed % 10U),
                 (unsigned)(total / 10U), (unsigned)(total % 10U));
    } else if (queued) {
        snprintf(phase, sizeof(phase), "queued");
    } else {
        snprintf(phase, sizeof(phase), "idle");
    }

    if (!atomic_load_explicit(&climate.event_valid, memory_order_relaxed)) {
        snprintf(out, len, "%s", phase);
        return true;
    }

    // the event and its timestamp come from two atomics; re-read so a frame
    // arriving in between cannot pair one id with another id's time
    uint32_t event = 0;
    uint32_t event_tick = 0;
    for (int attempt = 0; attempt < 3; attempt++) {
        event = atomic_load_explicit(&climate.event, memory_order_relaxed);
        event_tick = atomic_load_explicit(&climate.event_tick, memory_order_relaxed);
        if (event == atomic_load_explicit(&climate.event, memory_order_relaxed)) {
            break;
        }
    }

    const unsigned id = (unsigned)(event & 0x7FFU);
    const unsigned first = (unsigned)((event >> 16) & 0xFFU);
    const unsigned second = (unsigned)((event >> 24) & 0xFFU);
    const char *bus = (event & CLIMATE_EVENT_HU) != 0U ? "hu" : "car";

    if (started) {
        const int32_t delta = (int32_t)(event_tick - start_tick);
        const uint32_t since = delta > 0 ? (uint32_t)delta * 4U / 10U : 0U;
        snprintf(out, len, "%s; 0x%03X %02X %02X %s +%u.%u s", phase, id, first,
                 second, bus, (unsigned)(since / 10U), (unsigned)(since % 10U));
    } else {
        snprintf(out, len, "%s; 0x%03X %02X %02X %s", phase, id, first, second, bus);
    }
    return true;
}

#else  // !HW_HAS_MCP2515

// Single-bus boards have no head-unit bus to filter and no car bus to inject
// on, so the feature is compiled out and its call sites stay unconditional.

void climate_tick(void) {}

fwd_result_t climate_fwd_hook(twai_message_t *msg, can_bus_t fwd_bus) {
    (void)msg;
    (void)fwd_bus;
    return FWD_PASSTHROUGH;
}

esp_err_t climate_request_start(void) {
    return ESP_ERR_NOT_SUPPORTED;
}

bool climate_is_active(void) {
    return false;
}

void climate_rx_hook(const twai_message_t *msg, can_bus_t rx_bus) {
    (void)msg;
    (void)rx_bus;
}

bool climate_get_status(char *out, size_t len) {
    (void)out;
    (void)len;
    return false;
}

#endif
