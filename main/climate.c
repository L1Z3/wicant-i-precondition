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
// Reverse-engineered from the head unit's own traffic on the second bus
// (WiCAN_MITM/M-CAN_MITM_climate_start_nothing_62F_1.csv, bus 1) and confirmed
// against the stop capture, a charging capture with no remote request, and a
// utility-mode capture. A remote climate start is:
//
//   t+0.000s  0x4F1  00 D0 00 00 00 00 00 00   wake/session bit, x3
//   t+1.000s  0x4A2  00 00 0F 03 00 00 00 00   request (climate), x3
//   t+1.120s  0x4A2  00 00 FF 03 FF FF 00 00   apply, x3
//             0x0C7  00 00 00 01 00 00 00 00   climate opcode, x3
//   t+1.240s  0x0C7  00 00 00 00 00 00 00 00   trailer, x3
//   ...       0x4F1  00 D0 ...                kept alive until the car is up
//
// 0x4F1 byte 1 bit 4 (0xC0 -> 0xD0) is the only frame that precedes the
// command, and it is the one that wakes the car's climate ECUs: they start
// transmitting 0.4 s after it and 0.65 s before 0x4A2 arrives. 0x4A2 byte 2 is
// the request mask (0x0F climate only, 0xDF climate and seat warmers, 0xF3
// seats), byte 3 is 0x03 here (0x43/0xC3 on the lock/unlock commands).
//
// None of these frames carries authentication, a counter, or a seed/key
// exchange: 0x4A2 bytes 0-1 are 0x00 in every payload ever captured, the
// payloads are byte-identical across model years and head units, and no 0x7xx
// diagnostic traffic appears in any remote-climate capture. The three-frame
// repetition is the same latch the 0x0C7 preconditioning commands use.
//
// 0x0C7 byte 3 = 0x01 is the head unit's climate opcode. It is not modelled in
// the DBC and its effect is unknown; the kit only ever sent 0x40/0xE0/0x00, so
// this opcode is replayed exactly as captured.
//
// ********************* what the car answers *********************
//
// The captures also show which frames move when the car accepts the request.
// The Status tab reports the last one it sees, because none of them is
// documented and their exact meaning is still being worked out:
//
//   0x4EB  byte 4 bit 7    head unit's active flag. 0x00 -> 0x80 at 8.34 s in
//                          the start capture, ~1.1 s after the apply burst;
//                          already 0x80 in the charging and utility captures.
//   0x4DD  byte 2          state bit. 0xFE -> 0x00 at the same instant as the
//                          0x4A2 apply.
//   0x380  bytes 3..6      DATC setpoints (car bus). 00 00 00 00 while the
//                          climate unit is asleep, 06 06 06 06 once it is up.
//   0x4A2  bytes 2..3      the request itself, seen on either bus (the car
//                          sends its own 0x4A2 with byte 2 = 0x01).
//   0x0C7  bytes 3..4      the climate opcode and its trailer, echoed back.
//   0x4F1  byte 1          the wake bit, on while a remote session is open.
//   0x2AD  byte 1          BMS precondition state; moves when a remote
//                          request also starts the battery conditioning.

#define CLIMATE_CAR_BUS CAN_BUS_0

#define CLIMATE_WAKE_FRAME_ID 0x4F1U
#define CLIMATE_REQUEST_FRAME_ID 0x4A2U
#define CLIMATE_STATUS_FRAME_ID 0x0C7U

// Every command is repeated three times, matching the head unit and the 0x0C7
// burst helper in precondition.c.
#define CLIMATE_BURST_TICKS 3U

// The head unit holds the wake bit for a full second before it sends 0x4A2.
#define CLIMATE_WAKE_LEAD_TICKS 25U

// Keep the wake bit alive after the trailer so the car finishes waking its
// climate ECUs even if the head unit's own session frames are still blocked.
#define CLIMATE_HOLD_TICKS 125U

// The head unit re-asserts the wake bit about once a second.
#define CLIMATE_KEEPALIVE_TICKS 25U

#define CLIMATE_TRANSITION_TICKS \
    (CLIMATE_WAKE_LEAD_TICKS + 3U * CLIMATE_BURST_TICKS)
#define CLIMATE_SEQUENCE_TICKS (CLIMATE_TRANSITION_TICKS + CLIMATE_HOLD_TICKS)

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
} climate_state_t;

static climate_state_t climate;

// Event word: id in bits 0..14, above that the two payload bytes worth
// reporting, then which bus it arrived on, so one atomic load is a complete
// reading (the id is 11 bits, so bit 15 is free).
#define CLIMATE_EVENT_HU 0x8000U

// ********************* payloads *********************

static const uint8_t wake_data[8] = {0x00U, 0xD0U, 0x00U, 0x00U,
                                     0x00U, 0x00U, 0x00U, 0x00U};
static const uint8_t request_data[8] = {0x00U, 0x00U, 0x0FU, 0x03U,
                                        0x00U, 0x00U, 0x00U, 0x00U};
static const uint8_t apply_data[8] = {0x00U, 0x00U, 0xFFU, 0x03U,
                                      0xFFU, 0xFFU, 0x00U, 0x00U};
static const uint8_t opcode_data[8] = {0x00U, 0x00U, 0x00U, 0x01U,
                                       0x00U, 0x00U, 0x00U, 0x00U};
static const uint8_t trailer_data[8] = {0x00U, 0x00U, 0x00U, 0x00U,
                                        0x00U, 0x00U, 0x00U, 0x00U};

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

static void send_request_frame(void) {
    send_climate_frame(CLIMATE_REQUEST_FRAME_ID, request_data);
}

static void send_apply_frames(void) {
    send_climate_frame(CLIMATE_REQUEST_FRAME_ID, apply_data);
    send_climate_frame(CLIMATE_STATUS_FRAME_ID, opcode_data);
}

static void send_trailer_frame(void) {
    send_climate_frame(CLIMATE_STATUS_FRAME_ID, trailer_data);
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
    {CLIMATE_WAKE_FRAME_ID, 1U, 0U},     // wake bit, stay-alive byte
    {CLIMATE_REQUEST_FRAME_ID, 2U, 3U},  // request mask, command byte
    {CLIMATE_STATUS_FRAME_ID, 3U, 4U},   // opcode, its trailer
    {0x4DDU, 2U, 4U},                    // state byte, mode
    {0x4EBU, 4U, 2U},                    // active flag, minutes
    {0x380U, 3U, 4U},                    // DATC setpoints
    {0x2ADU, 1U, 0U},                    // BMS precondition state
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
        if (msg->identifier == CLIMATE_STATUS_FRAME_ID && first == 0U
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
        ESP_LOGI(TAG, "remote climate start: replaying head unit sequence");
    }

    const uint32_t tick = atomic_fetch_add_explicit(
        &climate.tick, 1U, memory_order_relaxed);
    if (tick < CLIMATE_BURST_TICKS) {
        send_wake_frame();
    } else if (tick < CLIMATE_WAKE_LEAD_TICKS) {
        // hold the wake bit and keep the head unit's own frames blocked, the
        // same ~1 s lead the head unit leaves before it sends the request
    } else if (tick < CLIMATE_WAKE_LEAD_TICKS + CLIMATE_BURST_TICKS) {
        send_request_frame();
    } else if (tick < CLIMATE_WAKE_LEAD_TICKS + 2U * CLIMATE_BURST_TICKS) {
        send_apply_frames();
    } else if (tick < CLIMATE_TRANSITION_TICKS) {
        send_trailer_frame();
    } else {
        const uint32_t hold_tick = tick - CLIMATE_TRANSITION_TICKS;
        if ((hold_tick % CLIMATE_KEEPALIVE_TICKS) < CLIMATE_BURST_TICKS) {
            send_wake_frame();
        }
        if (hold_tick + 1U >= CLIMATE_HOLD_TICKS) {
            atomic_store_explicit(&climate.active, false, memory_order_relaxed);
            ESP_LOGI(TAG, "remote climate start: sequence finished");
        }
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
    // not for the 6.3 s the sequence actually takes.
    char phase[40];
    if (active) {
        const uint32_t elapsed = tick * 4U / 10U;
        const uint32_t total = (uint32_t)CLIMATE_SEQUENCE_TICKS * 4U / 10U;
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
