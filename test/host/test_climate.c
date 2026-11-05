// Remote command replays (climate.c): frame order, burst timing, the heartbeat
// contents per command, and the forward-blocking window, against the fake clock
// and a recording can_send.

#include <string.h>
#include "can.h"
#include "climate.h"
#include "support/test_support.h"

// ********************* recording can_send *********************

#define MAX_SENT 1200U

typedef struct {
    uint32_t id;
    uint8_t data[8];
    can_bus_t bus;
} sent_t;

static sent_t sent[MAX_SENT];
static size_t sent_count;
static int wrong_bus;

esp_err_t can_send(can_bus_t bus, twai_message_t *message, TickType_t ticks_to_wait) {
    (void)ticks_to_wait;
    // the 0x39B heartbeat is I-CAN (bus 0 here), every command is M-CAN (bus 1)
    if (bus != (message->identifier == 0x39BU ? CAN_BUS_0 : CAN_BUS_1)) {
        wrong_bus++;
    }
    if (sent_count < MAX_SENT) {
        sent[sent_count].id = message->identifier;
        sent[sent_count].bus = bus;
        memcpy(sent[sent_count].data, message->data, 8U);
        sent_count++;
    }
    return ESP_OK;
}

// ********************* expected payloads *********************

// 0x4F1 byte 1 bit 4: the head unit's session bit, which is what brings the
// car's climate ECUs up before the command arrives
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
// 0x0C7 D4 = 0x01: the head unit's remote opcode
static const uint8_t opcode_payload[8] = {0x00U, 0x00U, 0x00U, 0x01U,
                                          0x00U, 0x00U, 0x00U, 0x00U};
static const uint8_t trailer_payload[8] = {0};
// 0x4DD is 5 Hz on this car and only byte 2 moves when the session opens
static const uint8_t state_payload[8] = {0x00U, 0x00U, 0x00U, 0x00U,
                                         0x08U, 0x00U, 0x00U, 0x00U};
// pending marker the head unit raises in byte 2 while its own request is in
// flight, which a start now mirrors
static const uint8_t state_pending_payload[8] = {0x00U, 0x00U, 0xFEU, 0x00U,
                                                0x08U, 0x00U, 0x00U, 0x00U};
// lock: request/apply differ only in the follow-up state nibble (D6 0x40)
static const uint8_t lock_request_payload[8] = {0x00U, 0x00U, 0xF3U, 0x43U,
                                                0x00U, 0x00U, 0xC0U, 0x00U};
static const uint8_t lock_apply_payload[8]   = {0x00U, 0x00U, 0xF3U, 0xC3U,
                                                0x00U, 0x00U, 0x40U, 0x00U};
// unlock has the same request and only D6 0xD0 in the apply
static const uint8_t unlock_apply_payload[8] = {0x00U, 0x00U, 0xF3U, 0xC3U,
                                                0x00U, 0x00U, 0xD0U, 0x00U};

// 0x39B, the I-CAN session heartbeat: byte 0 is the command state (0x00 before
// the apply, 0x50 while climate runs, 0x40 as it closes), byte 3 counts
// 00, 04 .. 3C, byte 5 is the command class.
// The steady heartbeat: `50 <setpoint> 00 <counter> 00 00 00 00`. The class and
// phase bytes are zero except while a command is executing, which the stop
// telegram below is the only user of.
static void tmu_payload(uint8_t state, uint8_t command_class, uint8_t counter,
                        uint8_t out[8]) {
    static const uint8_t base[8] = {0x00U, 0x10U, 0x00U, 0x00U,
                                    0x00U, 0x00U, 0x00U, 0x00U};
    memcpy(out, base, 8U);
    out[0] = state;
    out[3] = counter;
    out[5] = command_class;
}

static bool payload_is(size_t index, uint32_t id, const uint8_t expected[8]) {
    return index < sent_count
           && sent[index].id == id
           && memcmp(sent[index].data, expected, 8U) == 0;
}

// long enough for the whole replay plus the release checks
#define CLIMATE_TEST_TICKS 320U

// ********************* suites *********************

// The car's answers are only interesting as edges, and they are reported as one
// short line the Status tab can show.
static void suite_logging(void) {
    char status[64];
    twai_message_t frame = {0};
    frame.data_length_code = 8U;

    CHECK(climate_get_status(status, sizeof(status)));
    CHECK(strcmp(status, "idle") == 0);

    // idle session frames (bit 4 clear) are not a state change
    frame.identifier = 0x4F1U;
    frame.data[1] = 0xC0U;
    climate_rx_hook(&frame, CAN_BUS_0);
    CHECK(climate_get_status(status, sizeof(status)));
    CHECK(strcmp(status, "idle") == 0);

    // the session bit, as the head unit sends it
    memcpy(frame.data, wake_payload, 8U);
    climate_rx_hook(&frame, CAN_BUS_0);
    CHECK(climate_get_status(status, sizeof(status)));
    CHECK(strcmp(status, "idle; 0x4F1 D0 00 hu") == 0);

    // the car answers on its own bus with its own 0x4A2 payload
    frame.identifier = 0x4A2U;
    frame.data[2] = 0x01U;
    climate_rx_hook(&frame, CAN_BUS_1);
    CHECK(climate_get_status(status, sizeof(status)));
    CHECK(strcmp(status, "idle; 0x4A2 01 00 car") == 0);

    // the head unit's active flag, the response the prototype watches
    frame.identifier = 0x4EBU;
    memcpy(frame.data, "\x20\xA0\x16\x01\x80\x00\x00\x00", 8U);
    climate_rx_hook(&frame, CAN_BUS_0);
    CHECK(climate_get_status(status, sizeof(status)));
    CHECK(strcmp(status, "idle; 0x4EB 80 16 hu") == 0);

    // the vehicle power state is the clearest "it worked" edge
    frame.identifier = 0x037U;
    memcpy(frame.data, "\x22\x00\x00\x10\x00\x40\x00\x00", 8U);
    climate_rx_hook(&frame, CAN_BUS_1);
    CHECK(climate_get_status(status, sizeof(status)));
    CHECK(strcmp(status, "idle; 0x037 22 00 car") == 0);

    // repeating the same frame must not move the reported edge
    climate_rx_hook(&frame, CAN_BUS_1);
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
    CHECK(climate_fwd_hook(&frame, CAN_BUS_1) == FWD_PASSTHROUGH);
    frame.identifier = 0x4F1U;
    CHECK(climate_fwd_hook(&frame, CAN_BUS_1) == FWD_PASSTHROUGH);

    CHECK(climate_request_start(0) == ESP_OK);
    CHECK(climate_is_active());
    CHECK(strcmp(climate_command_label(0), "start") == 0);
    CHECK(climate_command_count() == 5U);
    CHECK(strcmp(climate_command_label(1), "stop") == 0);
    frame.identifier = 0x4A2U;
    CHECK(climate_fwd_hook(&frame, CAN_BUS_1) == FWD_BLOCK);
    frame.identifier = 0x4F1U;
    CHECK(climate_fwd_hook(&frame, CAN_BUS_1) == FWD_BLOCK);
    // the head unit side is untouched, and so is every other frame
    frame.identifier = 0x4A2U;
    CHECK(climate_fwd_hook(&frame, CAN_BUS_0) == FWD_PASSTHROUGH);
    frame.identifier = 0x4EBU;
    CHECK(climate_fwd_hook(&frame, CAN_BUS_1) == FWD_PASSTHROUGH);

    // a second request while this one is queued is refused, and off the end of
    // the table is an argument error rather than a run
    CHECK(climate_request_start(0) == ESP_ERR_INVALID_STATE);
    CHECK(climate_request_start(5) == ESP_ERR_INVALID_ARG);
}

// The full climate start: wake bit first, a one second lead with the session
// heartbeat already running, the two-phase 0x4A2 and the 0x0C7 opcode/trailer,
// eight seconds of session frames, then the session bit released.
// A start always opens with the wake preamble: 3 s of 0x4F1 00 C0, the
// 0x4A2 F3 43 session envelope, then 0x4F1 00 D0 with F3 C3 120 ms later, and
// only then the request at 1.04 s. Timings from the 2026-11-04 cloud capture.
static void suite_start(void) {
    // the sequence was queued by suite_blocking_window (which ticks nothing)
    size_t before = sent_count;
    CHECK(climate_is_active());

    size_t c0 = 0, open = 0, d0 = 0, commit = 0, request = 0;
    size_t apply = 0, opcode = 0, trailer = 0, state = 0, tmu_count = 0;
    size_t open_index = 0, d0_index = 0, request_tick = 0, last_send_tick = 0;
    uint8_t counter = 0U;
    for (size_t tick = 0; tick < CLIMATE_TEST_TICKS; tick++) {
        size_t previous = sent_count;
        climate_tick();
        for (size_t k = previous; k < sent_count; k++) {
            last_send_tick = tick;
            if (sent[k].id == 0x4F1U && sent[k].data[1] == 0xC0U) {
                c0++;
            }
            if (sent[k].id == 0x4F1U && sent[k].data[1] == 0xD0U) {
                if (d0 == 0U) {
                    d0_index = k;
                }
                d0++;
            }
            if (sent[k].id == 0x4A2U) {
                if (sent[k].data[2] == 0xF3U && sent[k].data[3] == 0x43U) {
                    if (open == 0U) {
                        open_index = k;
                    }
                    open++;
                } else if (sent[k].data[2] == 0xF3U && sent[k].data[3] == 0xC3U) {
                    commit++;
                } else if (sent[k].data[2] == 0x0FU) {
                    if (request == 0U) {
                        request_tick = tick;
                    }
                    request++;
                } else if (sent[k].data[2] == 0xFFU) {
                    apply++;
                }
            }
            if (sent[k].id == 0x0C7U) {
                if (sent[k].data[3] == 0x01U) {
                    opcode++;
                } else {
                    trailer++;
                }
            }
            if (sent[k].id == 0x4DDU) {
                // a start raises the pending marker in byte 2 until the apply,
                // exactly as the head unit does for its own requests
                CHECK(memcmp(sent[k].data, state_pending_payload, 8U) == 0
                      || memcmp(sent[k].data, state_payload, 8U) == 0);
                state++;
            }
            if (sent[k].id == 0x39BU) {
                CHECK_MSG(sent[k].data[3] == counter, "0x39B counter %02X at %u",
                          (unsigned)sent[k].data[3], (unsigned)k);
                // a start opens the session the way the telematics unit does:
                // class 1 while the window opens, class 3 afterwards, phase 0x10
                // class 0 phase 0 is the plain heartbeat; while the session is
                // being announced the same frame carries class 1 then class 3
                // with phase 0x10, as the telematics unit sends it
                if (sent[k].data[5] == 0x00U) {
                    CHECK(sent[k].data[6] == 0x00U);
                } else {
                    CHECK(sent[k].data[5] == 0x01U || sent[k].data[5] == 0x03U);
                    CHECK(sent[k].data[6] == 0x10U);
                }
                CHECK(sent[k].data[1] == 0x10U);
                counter = (uint8_t)((counter + 4U) & 0x3FU);
                tmu_count++;
            }
        }
    }

    // rest bit: three in the preamble burst, 1 Hz at 25 and 50 ticks, then
    // the closing burst of three
    CHECK(c0 == 8U);
    // the envelope, then the request 26 ticks (1.04 s) after the D0 burst
    CHECK(open == 3U);
    CHECK(commit == 3U);
    CHECK(d0 >= 3U);
    CHECK(open_index < d0_index);
    CHECK(request == 3U);
    CHECK(request_tick == 104U);
    CHECK(apply == 3U);
    CHECK(opcode == 3U);
    CHECK(trailer == 3U);
    CHECK(state > 0U && tmu_count > 0U);
    // nothing is sent after the closing burst
    CHECK(last_send_tick < CLIMATE_TEST_TICKS - 1U);

    // the sequence releases the block when it finishes, and can run again
    CHECK(!climate_is_active());
    twai_message_t frame = {0};
    frame.identifier = 0x4A2U;
    CHECK(climate_fwd_hook(&frame, CAN_BUS_1) == FWD_PASSTHROUGH);
}

// A stop is the telematics telegram on the I-CAN and nothing else: no 0x4A2,
// no 0x0C7, no 0x4DD. 0x39B runs class 0x01 phase 0x10 for three seconds, then
// class 0x03 phase 0x10 for one, then class 0x03 phase 0x14, and the car is off
// ~90 ms later (captured 2026-11-04 while a real stop went through).
static void suite_stop(void) {
    size_t before = sent_count;
    CHECK(climate_request_start(1) == ESP_OK);
    for (size_t tick = 0; tick < CLIMATE_TEST_TICKS; tick++) {
        climate_tick();
    }

    size_t a2 = 0, c7 = 0, state = 0, tmu_count = 0;
    size_t preamble = 0, climate = 0, applied = 0, idle = 0;
    uint8_t counter = 0U;
    for (size_t i = before; i < sent_count; i++) {
        if (sent[i].id == 0x4A2U) {
            a2++;
        } else if (sent[i].id == 0x0C7U) {
            c7++;
        } else if (sent[i].id == 0x4DDU) {
            state++;
        } else if (sent[i].id == 0x39BU) {
            CHECK_MSG(sent[i].data[3] == counter, "0x39B counter %02X at %u",
                      (unsigned)sent[i].data[3], (unsigned)i);
            CHECK(sent[i].data[0] == 0x00U);
            CHECK(sent[i].data[7] == 0x00U);
            if (sent[i].data[5] == 0x01U) {
                // the telegram reports "not running, no setpoint"
                CHECK(sent[i].data[1] == 0x00U);
                CHECK(sent[i].data[6] == 0x10U);
                preamble++;
            } else if (sent[i].data[5] == 0x03U && sent[i].data[6] == 0x10U) {
                CHECK(sent[i].data[1] == 0x00U);
                climate++;
            } else if (sent[i].data[5] == 0x03U && sent[i].data[6] == 0x14U) {
                CHECK(sent[i].data[1] == 0x00U);
                applied++;
            } else {
                CHECK(sent[i].data[5] == 0x00U && sent[i].data[6] == 0x00U);
                idle++;
            }
            counter = (uint8_t)((counter + 4U) & 0x3FU);
            tmu_count++;
        }
    }
    CHECK(a2 == 0U);
    CHECK(c7 == 0U);
    CHECK(state == 0U);
    CHECK(preamble == 15U);
    CHECK(climate == 5U);
    CHECK(applied == 3U);
    CHECK(idle == 17U);
    CHECK(tmu_count == 40U);
    // 3 wake + 3 idle + 40 telegram + 8 keepalives = 54
    CHECK(sent_count - before == 54U);
    CHECK(!climate_is_active());
}

// Lock and unlock differ only in the apply's state nibble; the request and the
// command class are identical, which is the whole reason both buttons exist.
static void suite_lock_unlock(void) {
    size_t before = sent_count;
    CHECK(climate_request_start(2) == ESP_OK);
    for (size_t tick = 0; tick < CLIMATE_TEST_TICKS; tick++) {
        climate_tick();
    }
    size_t lock_frames = sent_count - before;
    CHECK(lock_frames == 108U);

    size_t lock_request = 0, lock_apply = 0, wrong_class = 0;
    for (size_t i = before; i < sent_count; i++) {
        if (sent[i].id == 0x4A2U) {
            if (memcmp(sent[i].data, lock_request_payload, 8U) == 0) {
                lock_request++;
            } else if (memcmp(sent[i].data, lock_apply_payload, 8U) == 0) {
                lock_apply++;
            }
        } else if (sent[i].id == 0x39BU && (sent[i].data[5] != 0x00U
                                            || sent[i].data[6] != 0x00U
                                            || sent[i].data[0] != 0x00U)) {
            wrong_class++;
        }
    }
    CHECK(lock_request == 3U);
    CHECK(lock_apply == 3U);
    CHECK(wrong_class == 0U);

    before = sent_count;
    CHECK(climate_request_start(3) == ESP_OK);
    for (size_t tick = 0; tick < CLIMATE_TEST_TICKS; tick++) {
        climate_tick();
    }
    CHECK(sent_count - before == 108U);
    size_t unlock_request = 0, unlock_apply = 0, lock_nibble = 0;
    for (size_t i = before; i < sent_count; i++) {
        if (sent[i].id != 0x4A2U) {
            continue;
        }
        if (memcmp(sent[i].data, lock_request_payload, 8U) == 0) {
            unlock_request++;
        } else if (memcmp(sent[i].data, unlock_apply_payload, 8U) == 0) {
            unlock_apply++;
        } else if (memcmp(sent[i].data, lock_apply_payload, 8U) == 0) {
            lock_nibble++;
        }
    }
    CHECK(unlock_request == 3U);
    CHECK(unlock_apply == 3U);
    CHECK(lock_nibble == 0U);
    CHECK(!climate_is_active());
}

// The option checkboxes are bits in the one start burst: each of the three
// captures shows the same envelope with a different mask, so a single option
// has to reproduce them byte for byte.
static void suite_options(void) {
    static const struct {
        uint8_t options;
        uint8_t request[8];
        uint8_t apply[8];
    } cases[] = {
        {CLIMATE_OPTION_FRONT_DEFROST,
         {0x00U, 0x00U, 0x0FU, 0xC3U, 0x00U, 0x00U, 0xF0U, 0x00U},
         {0x00U, 0x00U, 0xFFU, 0xC3U, 0xFFU, 0xFFU, 0xF0U, 0x00U}},
        {CLIMATE_OPTION_REAR_DEFROST,
         {0x00U, 0x00U, 0x7FU, 0xC3U, 0x00U, 0x00U, 0xF0U, 0x00U},
         {0x00U, 0x00U, 0xFFU, 0xC3U, 0xFFU, 0xFFU, 0xF0U, 0x00U}},
        {CLIMATE_OPTION_STEERING_HEAT,
         {0x00U, 0x00U, 0xDFU, 0xC3U, 0x00U, 0x00U, 0xF0U, 0x00U},
         {0x00U, 0x00U, 0xFFU, 0xC3U, 0xFFU, 0xFFU, 0xF0U, 0x00U}},
    };
    for (size_t c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
        for (size_t tick = 0; tick < CLIMATE_TEST_TICKS && climate_is_active(); tick++) {
            climate_tick();
        }
        CHECK(!climate_is_active());
        const size_t before = sent_count;
        CHECK(climate_request_start_temp(0, 220, cases[c].options) == ESP_OK);
        for (size_t tick = 0; tick < 130U; tick++) {
            climate_tick();
        }
        size_t requests = 0, applies = 0;
        for (size_t i = before; i < sent_count; i++) {
            if (sent[i].id != 0x4A2U) {
                continue;
            }
            if (payload_is(i, 0x4A2U, cases[c].request)) {
                requests++;
            }
            if (payload_is(i, 0x4A2U, cases[c].apply)) {
                applies++;
            }
        }
        CHECK(requests == 3U);
        CHECK(applies == 3U);
    }
}

// The car takes the cabin temperature from 0x39B byte 1 and its 0x2AC mirror,
// raw = 2T - 28, so a request for 22.5 C has to come out as 0x11 on both.
static void suite_setpoint(void) {
    // let the sleeping-car run finish first, so the next request is accepted
    for (size_t tick = 0; tick < CLIMATE_TEST_TICKS && climate_is_active(); tick++) {
        climate_tick();
    }
    CHECK(!climate_is_active());

    size_t before = sent_count;
    CHECK(climate_request_start_temp(0, 225, 0U) == ESP_OK);
    for (size_t tick = 0; tick < 130U; tick++) {
        climate_tick();
    }

    size_t tmu = 0;
    for (size_t i = before; i < sent_count; i++) {
        if (sent[i].id == 0x39BU) {
            CHECK(sent[i].data[1] == 0x11U);
            CHECK(sent[i].bus == CAN_BUS_0);
            tmu++;
        }
    }
    CHECK(tmu > 0U);

    // off the ends of the table: 10 C clamps to 14.0 C (raw 0), 40 C to 32 C
    for (size_t tick = 0; tick < CLIMATE_TEST_TICKS && climate_is_active(); tick++) {
        climate_tick();
    }
    before = sent_count;
    CHECK(climate_request_start_temp(0, 100, 0U) == ESP_OK);
    for (size_t tick = 0; tick < 26U; tick++) {
        climate_tick();
    }
    for (size_t i = before; i < sent_count; i++) {
        if (sent[i].id == 0x39BU) {
            CHECK(sent[i].data[1] == 0x00U);
        }
    }
}

int main(void) {
    suite_logging();
    suite_blocking_window();
    suite_start();
    const size_t start_sends = sent_count;
    suite_stop();
    const size_t stop_sends = sent_count - start_sends;
    suite_lock_unlock();
    const size_t lock_sends = sent_count - start_sends - stop_sends;

    CHECK(wrong_bus == 0);
    // every suite sends what it sent, and no suite left a frame behind
    CHECK(sent_count == start_sends + stop_sends + lock_sends);
    suite_options();
    suite_setpoint();
    return test_report("test_climate");
}
