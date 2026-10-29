// Remote climate start replay (climate.c): frame order, burst timing, and the
// forward blocking window, against the fake clock and a recording can_send.

#include <string.h>
#include "can.h"
#include "climate.h"
#include "support/test_support.h"

// ********************* recording can_send *********************

#define MAX_SENT 64U

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

static bool payload_is(size_t index, uint32_t id, const uint8_t expected[8]) {
    return index < sent_count
           && sent[index].id == id
           && memcmp(sent[index].data, expected, 8U) == 0;
}

// long enough for the whole replay plus the release checks
#define CLIMATE_TEST_TICKS 200U

// ********************* suites *********************

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

// The full replay: wake bit first, a one second lead, the two-phase 0x4A2 and
// the 0x0C7 opcode/trailer, then the wake bit held once a second.
static void suite_sequence(void) {
    size_t per_tick[CLIMATE_TEST_TICKS];
    size_t before = sent_count;

    for (size_t tick = 0; tick < CLIMATE_TEST_TICKS; tick++) {
        size_t previous = sent_count;
        climate_tick();
        per_tick[tick] = sent_count - previous;
    }

    // ticks 0..2 wake burst, then nothing until the lead time elapses
    for (size_t tick = 0; tick < 3U; tick++) {
        CHECK_MSG(per_tick[tick] == 1U, "tick %u sent %u", (unsigned)tick,
                  (unsigned)per_tick[tick]);
    }
    for (size_t tick = 3U; tick < 25U; tick++) {
        CHECK_MSG(per_tick[tick] == 0U, "tick %u sent %u", (unsigned)tick,
                  (unsigned)per_tick[tick]);
    }

    // 25..27 request, 28..30 apply (0x4A2 and 0x0C7 together), 31..33 trailer
    for (size_t tick = 25U; tick < 28U; tick++) {
        CHECK(per_tick[tick] == 1U);
    }
    for (size_t tick = 28U; tick < 31U; tick++) {
        CHECK(per_tick[tick] == 2U);
    }
    for (size_t tick = 31U; tick < 34U; tick++) {
        CHECK(per_tick[tick] == 1U);
    }

    size_t index = before;
    CHECK(payload_is(index++, 0x4F1U, wake_payload));
    CHECK(payload_is(index++, 0x4F1U, wake_payload));
    CHECK(payload_is(index++, 0x4F1U, wake_payload));
    for (int k = 0; k < 3; k++) {
        CHECK(payload_is(index++, 0x4A2U, request_payload));
    }
    for (int k = 0; k < 3; k++) {
        CHECK(payload_is(index++, 0x4A2U, apply_payload));
        CHECK(payload_is(index++, 0x0C7U, opcode_payload));
    }
    for (int k = 0; k < 3; k++) {
        CHECK(payload_is(index++, 0x0C7U, trailer_payload));
    }

    // hold: one three-frame wake burst a second, nothing in between
    CHECK(per_tick[34] == 1U && per_tick[35] == 1U && per_tick[36] == 1U);
    CHECK(per_tick[58] == 0U && per_tick[59] == 1U && per_tick[62] == 0U);
    for (size_t tick = 36U; tick < 159U; tick++) {
        size_t offset = (tick - 34U) % 25U;
        CHECK_MSG(per_tick[tick] == (offset < 3U ? 1U : 0U), "tick %u",
                  (unsigned)tick);
    }

    // the sequence releases the block when it finishes, and can run again
    for (size_t tick = 159U; tick < CLIMATE_TEST_TICKS; tick++) {
        CHECK(per_tick[tick] == 0U);
    }
    CHECK(!climate_is_active());
    twai_message_t frame = {0};
    frame.identifier = 0x4A2U;
    CHECK(climate_fwd_hook(&frame, CAN_BUS_0) == FWD_PASSTHROUGH);
    CHECK(climate_request_start() == ESP_OK);
    CHECK(climate_is_active());
}

int main(void) {
    suite_blocking_window();
    suite_sequence();

    CHECK(wrong_bus == 0);
    // 3 wake + 3 request + 6 apply + 3 trailer + 5 keepalive bursts
    CHECK(sent_count == 30U);
    return test_report("test_climate");
}
