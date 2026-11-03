// Remote climate start replay (climate.c): frame order, burst timing, and the
// forward blocking window, against the fake clock and a recording can_send.

#include <string.h>
#include "can.h"
#include "climate.h"
#include "support/test_support.h"

// ********************* recording can_send *********************

#define MAX_SENT 160U

typedef struct {
    uint32_t id;
    uint8_t data[8];
} sent_t;

static sent_t sent[MAX_SENT];
static size_t sent_count;
static int wrong_bus;

esp_err_t can_send(can_bus_t bus, twai_message_t *message, TickType_t ticks_to_wait) {
    (void)ticks_to_wait;
    if (bus != CAN_BUS_0) {
        wrong_bus++;
    }
    if (sent_count < MAX_SENT) {
        sent[sent_count].id = message->identifier;
        memcpy(sent[sent_count].data, message->data, 8U);
        sent_count++;
    }
    return ESP_OK;
}

// ********************* expected payloads *********************

// 0x4F1 byte 1 bit 4: the head unit's wake bit, which is what brings the car's
// climate ECUs up before the command arrives
static const uint8_t wake_payload[8]  = {0x00U, 0xD0U, 0x00U, 0x00U,
                                         0x00U, 0x00U, 0x00U, 0x00U};
// 0x4F1 closing the session, as the head unit does 8 s later
static const uint8_t idle_payload[8]  = {0x00U, 0xC0U, 0x00U, 0x00U,
                                         0x00U, 0x00U, 0x00U, 0x00U};
// 0x4A2 D2 = 0x0F (climate only), D3 = 0x03
static const uint8_t request_payload[8] = {0x00U, 0x00U, 0x0FU, 0x03U,
                                           0x00U, 0x00U, 0x00U, 0x00U};
// 0x4A2 apply phase: D2 = 0xFF, D4/D5 = 0xFF 0xFF
static const uint8_t apply_payload[8] = {0x00U, 0x00U, 0xFFU, 0x03U,
                                         0xFFU, 0xFFU, 0x00U, 0x00U};
// 0x0C7 D4 = 0x01: the head unit's climate opcode
static const uint8_t opcode_payload[8] = {0x00U, 0x00U, 0x00U, 0x01U,
                                          0x00U, 0x00U, 0x00U, 0x00U};
static const uint8_t trailer_payload[8] = {0};
// 0x4DD is 5 Hz on this car and only byte 2 moves when the session opens
static const uint8_t state_payload[8] = {0x00U, 0x00U, 0x00U, 0x00U,
                                         0x08U, 0x00U, 0x00U, 0x00U};
// 0x39B, the I-CAN session heartbeat: byte 0 is 0x00 before the apply, 0x50
// while running, byte 3 counts 00, 04 .. 3C
static void tmu_payload(bool running, uint8_t counter, uint8_t out[8]) {
    static const uint8_t base[8] = {0x00U, 0x06U, 0x00U, 0x00U,
                                    0x00U, 0x03U, 0x10U, 0x00U};
    memcpy(out, base, 8U);
    out[0] = (uint8_t)(running ? 0x50U : 0x00U);
    out[3] = counter;
}

static bool payload_is(size_t index, uint32_t id, const uint8_t expected[8]) {
    return index < sent_count
           && sent[index].id == id
           && memcmp(sent[index].data, expected, 8U) == 0;
}

// long enough for the whole replay plus the release checks
#define CLIMATE_TEST_TICKS 230U

// ********************* suites *********************

// The car's answers are only interesting as edges, and they are reported as one
// short line the Status tab can show.
static void suite_logging(void) {
    char status[64];
    twai_message_t frame = {0};
    frame.data_length_code = 8U;

    CHECK(climate_get_status(status, sizeof(status)));
    CHECK(strcmp(status, "idle") == 0);

    // idle wake frames (bit 4 clear) are not a state change
    frame.identifier = 0x4F1U;
    frame.data[1] = 0xC0U;
    climate_rx_hook(&frame, CAN_BUS_1);
    CHECK(climate_get_status(status, sizeof(status)));
    CHECK(strcmp(status, "idle") == 0);

    // the wake bit, as the head unit sends it
    memcpy(frame.data, wake_payload, 8U);
    climate_rx_hook(&frame, CAN_BUS_1);
    CHECK(climate_get_status(status, sizeof(status)));
    CHECK(strcmp(status, "idle; 0x4F1 D0 00 hu") == 0);

    // the car answers on its own bus with its own 0x4A2 payload
    frame.identifier = 0x4A2U;
    frame.data[2] = 0x01U;
    climate_rx_hook(&frame, CAN_BUS_0);
    CHECK(climate_get_status(status, sizeof(status)));
    CHECK(strcmp(status, "idle; 0x4A2 01 00 car") == 0);

    // the head unit's active flag, the response the prototype watches
    frame.identifier = 0x4EBU;
    memcpy(frame.data, "\x20\xA0\x16\x01\x80\x00\x00\x00", 8U);
    climate_rx_hook(&frame, CAN_BUS_1);
    CHECK(climate_get_status(status, sizeof(status)));
    CHECK(strcmp(status, "idle; 0x4EB 80 16 hu") == 0);

    // the vehicle power state is the clearest "it worked" edge
    frame.identifier = 0x037U;
    memcpy(frame.data, "\x22\x00\x00\x10\x00\x40\x00\x00", 8U);
    climate_rx_hook(&frame, CAN_BUS_0);
    CHECK(climate_get_status(status, sizeof(status)));
    CHECK(strcmp(status, "idle; 0x037 22 00 car") == 0);

    // repeating the same frame must not move the reported edge
    climate_rx_hook(&frame, CAN_BUS_0);
    CHECK(climate_get_status(status, sizeof(status)));
    CHECK(strcmp(status, "idle; 0x037 22 00 car") == 0);
}

// A fresh request is invisible until it is queued, then blocks both frames on
// the way to the car (and only to the car) before the tick picks it up.
static void suite_blocking_window(void) {
    twai_message_t frame = {0};

    for (int i = 0; i < 10; i++) {
        climate_tick();
    }
    CHECK(sent_count == 0);
    CHECK(!climate_is_active());
    frame.identifier = 0x4A2U;
    CHECK(climate_fwd_hook(&frame, CAN_BUS_0) == FWD_PASSTHROUGH);
    frame.identifier = 0x4F1U;
    CHECK(climate_fwd_hook(&frame, CAN_BUS_0) == FWD_PASSTHROUGH);

    CHECK(climate_request_start() == ESP_OK);
    CHECK(climate_is_active());
    frame.identifier = 0x4A2U;
    CHECK(climate_fwd_hook(&frame, CAN_BUS_0) == FWD_BLOCK);
    frame.identifier = 0x4F1U;
    CHECK(climate_fwd_hook(&frame, CAN_BUS_0) == FWD_BLOCK);
    // the head unit side is untouched, and so is every other frame
    frame.identifier = 0x4A2U;
    CHECK(climate_fwd_hook(&frame, CAN_BUS_1) == FWD_PASSTHROUGH);
    frame.identifier = 0x4EBU;
    CHECK(climate_fwd_hook(&frame, CAN_BUS_0) == FWD_PASSTHROUGH);

    // a second request while this one is queued is refused
    CHECK(climate_request_start() == ESP_ERR_INVALID_STATE);
}

// The full replay: wake bit first, a one second lead with the session
// heartbeat already running, the two-phase 0x4A2 and the 0x0C7 opcode/trailer,
// eight seconds of session frames, then the wake bit released.
static void suite_sequence(void) {
    size_t per_tick[CLIMATE_TEST_TICKS];
    size_t before = sent_count;

    char status[64];
    for (size_t tick = 0; tick < CLIMATE_TEST_TICKS; tick++) {
        size_t previous = sent_count;
        climate_tick();
        per_tick[tick] = sent_count - previous;
        if (tick == 0U) {
            // the line says what is running and how the last response sat
            // relative to the start of the sequence
            CHECK(climate_get_status(status, sizeof(status)));
            CHECK(strstr(status, "replaying 0.0/8.3 s;") != NULL);
            CHECK(strstr(status, "0x037 22 00 car") != NULL);
        }
    }

    // ticks 0..2 wake burst, then the heartbeat starts two ticks later
    CHECK(per_tick[0] == 1U && per_tick[1] == 1U && per_tick[2] == 1U);
    CHECK(per_tick[3] == 1U && per_tick[4] == 1U);
    CHECK(per_tick[5] == 0U && per_tick[6] == 0U && per_tick[7] == 0U);
    // 0x4F1 re-asserts once a second
    CHECK(per_tick[11] == 1U);

    // 26..28 request, 29..31 apply (0x4A2 and 0x0C7 together), 32..34 trailer
    CHECK(per_tick[26] == 1U && per_tick[27] == 1U && per_tick[28] == 2U);
    CHECK(per_tick[29] == 3U && per_tick[30] == 2U && per_tick[31] == 2U);
    CHECK(per_tick[32] == 1U && per_tick[33] == 2U && per_tick[34] == 2U);

    // 205..207 close the session with the wake bit released
    CHECK(per_tick[205] == 1U && per_tick[206] == 1U && per_tick[207] == 1U);
    for (size_t tick = 208U; tick < CLIMATE_TEST_TICKS; tick++) {
        CHECK_MSG(per_tick[tick] == 0U, "tick %u sent %u", (unsigned)tick,
                  (unsigned)per_tick[tick]);
    }

    // the first twelve frames: burst, heartbeat, state, heartbeat, wake, ...
    size_t index = before;
    for (int k = 0; k < 3; k++) {
        CHECK(payload_is(index++, 0x4F1U, wake_payload));
    }
    uint8_t tmu[8];
    tmu_payload(false, 0x00U, tmu);
    CHECK(payload_is(index++, 0x39BU, tmu));
    CHECK(payload_is(index++, 0x4DDU, state_payload));
    tmu_payload(false, 0x04U, tmu);
    CHECK(payload_is(index++, 0x39BU, tmu));
    CHECK(payload_is(index++, 0x4DDU, state_payload));
    CHECK(payload_is(index++, 0x4F1U, wake_payload));

    // payload census
    size_t wake = 0, idle = 0, request = 0, apply = 0, opcode = 0;
    size_t trailer = 0, state = 0, tmu_count = 0, tmu_running = 0;
    uint8_t counter = 0U;
    for (size_t i = 0; i < sent_count; i++) {
        if (sent[i].id == 0x4F1U) {
            if (memcmp(sent[i].data, wake_payload, 8U) == 0) {
                wake++;
            } else if (memcmp(sent[i].data, idle_payload, 8U) == 0) {
                idle++;
            }
        } else if (sent[i].id == 0x4A2U) {
            if (memcmp(sent[i].data, request_payload, 8U) == 0) {
                request++;
            } else if (memcmp(sent[i].data, apply_payload, 8U) == 0) {
                apply++;
            }
        } else if (sent[i].id == 0x0C7U) {
            if (memcmp(sent[i].data, opcode_payload, 8U) == 0) {
                opcode++;
            } else if (memcmp(sent[i].data, trailer_payload, 8U) == 0) {
                trailer++;
            }
        } else if (sent[i].id == 0x4DDU) {
            if (memcmp(sent[i].data, state_payload, 8U) == 0) {
                state++;
            }
        } else if (sent[i].id == 0x39BU) {
            // the counter byte advances by four and wraps at 0x40
            CHECK_MSG(sent[i].data[3] == counter, "0x39B counter %02X at %u",
                      (unsigned)sent[i].data[3], (unsigned)i);
            CHECK(sent[i].data[1] == 0x06U && sent[i].data[5] == 0x03U
                  && sent[i].data[6] == 0x10U && sent[i].data[7] == 0x00U);
            CHECK(sent[i].data[0] == 0x00U || sent[i].data[0] == 0x50U);
            if (sent[i].data[0] == 0x50U) {
                tmu_running++;
            }
            counter = (uint8_t)((counter + 4U) & 0x3FU);
            tmu_count++;
        }
    }
    CHECK(wake == 11U);      // three-frame burst, then 1 Hz for eight seconds
    CHECK(idle == 3U);       // the closing 0x4F1 00 C0 burst
    CHECK(request == 3U);
    CHECK(apply == 3U);
    CHECK(opcode == 3U);
    CHECK(trailer == 3U);
    CHECK(state == 41U);     // 5 Hz for the whole session window
    CHECK(tmu_count == 41U);
    CHECK(tmu_running == 35U);

    // the sequence releases the block when it finishes, and can run again
    CHECK(!climate_is_active());
    twai_message_t frame = {0};
    frame.identifier = 0x4A2U;
    CHECK(climate_fwd_hook(&frame, CAN_BUS_0) == FWD_PASSTHROUGH);
    CHECK(climate_request_start() == ESP_OK);
    CHECK(climate_is_active());
}

int main(void) {
    suite_logging();
    suite_blocking_window();
    suite_sequence();

    CHECK(wrong_bus == 0);
    // 11 wake + 3 idle + 3 request + 3 apply + 3 opcode + 3 trailer
    // + 41 state + 41 heartbeat = 108
    CHECK(sent_count == 108U);
    return test_report("test_climate");
}
