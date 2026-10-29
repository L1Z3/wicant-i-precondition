#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
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
    uint32_t tick;
} climate_state_t;

static climate_state_t climate;

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

// ********************* public API *********************

static bool climate_blocking(void) {
    return atomic_load_explicit(&climate.requested, memory_order_relaxed)
           || atomic_load_explicit(&climate.active, memory_order_relaxed);
}

void climate_tick(void) {
    if (!atomic_load_explicit(&climate.active, memory_order_relaxed)) {
        if (!atomic_exchange_explicit(&climate.requested, false,
                                      memory_order_relaxed)) {
            return;
        }
        atomic_store_explicit(&climate.active, true, memory_order_relaxed);
        climate.tick = 0U;
        ESP_LOGI(TAG, "remote climate start: replaying head unit sequence");
    }

    const uint32_t tick = climate.tick++;
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
    (void)msg;
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

#endif
