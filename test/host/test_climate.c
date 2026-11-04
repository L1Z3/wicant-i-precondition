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
static void tmu_payload(uint8_t state, uint8_t command_class, uint8_t counter,
                        uint8_t out[8]) {
    static const uint8_t base[8] = {0x00U, 0x10U, 0x00U, 0x00U,
                                    0x00U, 0x03U, 0x10U, 0x00U};
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
    CHECK(climate_command_count() == 8U);
    CHECK(strcmp(climate_command_label(7), "start (asleep)") == 0);
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
    CHECK(climate_request_start(8) == ESP_ERR_INVALID_ARG);
}

// The full climate start: wake bit first, a one second lead with the session
// heartbeat already running, the two-phase 0x4A2 and the 0x0C7 opcode/trailer,
// eight seconds of session frames, then the session bit released.
// The sleeping-car variant: 3 s of 0x4F1 00 C0 (the rest bit), then the 0x4A2
// F3 43 session envelope, then 0x4F1 00 D0 with F3 C3 120 ms later, then the
// normal request at 1.04 s. Timings from the 2026-11-04 cloud capture.
static size_t asleep_sends;

static void suite_start_asleep(void) {
    CHECK(climate_request_start(7) == ESP_OK);

    size_t c0 = 0, open = 0, d0 = 0, commit = 0, request = 0;
    size_t open_index = 0, d0_index = 0, request_tick = 0;
    for (size_t tick = 0; tick < 130U; tick++) {
        size_t previous = sent_count;
        climate_tick();
        for (size_t k = previous; k < sent_count; k++) {
            if (sent[k].id == 0x4F1U && sent[k].data[1] == 0xC0U) {
                c0++;
            }
            if (sent[k].id == 0x4A2U && sent[k].data[2] == 0xF3U
                && sent[k].data[3] == 0x43U) {
                if (open == 0U) {
                    open_index = k;
                }
                open++;
            }
            if (sent[k].id == 0x4F1U && sent[k].data[1] == 0xD0U) {
                if (d0 == 0U) {
                    d0_index = k;
                }
                d0++;
            }
            if (sent[k].id == 0x4A2U && sent[k].data[2] == 0xF3U
                && sent[k].data[3] == 0xC3U) {
                commit++;
            }
            if (sent[k].id == 0x4A2U && sent[k].data[2] == 0x0FU) {
                if (request == 0U) {
                    request_tick = tick;
                }
                request++;
            }
        }
    }
    asleep_sends = sent_count - 420U;

    // ticks 0..2 rest burst, then 1 Hz at 25 and 50, then F3 43 at 75..77
    CHECK(c0 == 5U);
    CHECK(open == 3U);
    // 0x4F1 00 D0 and 0x4A2 F3 C3 arrive together, 120 ms after F3 43; the
    // 1 Hz session keepalive below is 0x4F1 00 D0 too, so only the burst and
    // the ordering are asserted
    CHECK(d0 >= 3U);
    CHECK(commit == 3U);
    CHECK(open_index < d0_index);
    // 3 s preamble + 6 ticks for the envelope + 26 ticks of wake lead
    CHECK(request == 3U);
    CHECK(request_tick == 104U);
}

static void suite_start(void) {
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
            CHECK(strstr(status, "replaying start 0.0/8.3 s;") != NULL);
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

    // 205..207 close the session with the session bit released
    CHECK(per_tick[205] == 1U && per_tick[206] == 1U && per_tick[207] == 1U);
    for (size_t tick = 208U; tick < CLIMATE_TEST_TICKS; tick++) {
        CHECK_MSG(per_tick[tick] == 0U, "tick %u sent %u", (unsigned)tick,
                  (unsigned)per_tick[tick]);
    }

    // the first frames: burst, heartbeat, state, heartbeat, wake, ...
    size_t index = before;
    for (int k = 0; k < 3; k++) {
        CHECK(payload_is(index++, 0x4F1U, wake_payload));
    }
    uint8_t tmu[8];
    tmu_payload(0x00U, 0x03U, 0x00U, tmu);
    CHECK(payload_is(index++, 0x39BU, tmu));
    CHECK(payload_is(index++, 0x4DDU, state_payload));
    tmu_payload(0x00U, 0x03U, 0x04U, tmu);
    CHECK(payload_is(index++, 0x39BU, tmu));
    CHECK(payload_is(index++, 0x4DDU, state_payload));
    CHECK(payload_is(index++, 0x4F1U, wake_payload));

    // payload census
    size_t wake = 0, idle = 0, request = 0, apply = 0, opcode = 0;
    size_t trailer = 0, state = 0, tmu_count = 0, tmu_running = 0;
    uint8_t counter = 0U;
    for (size_t i = before; i < sent_count; i++) {
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
            CHECK(memcmp(sent[i].data, state_payload, 8U) == 0);
            state++;
        } else if (sent[i].id == 0x39BU) {
            // the counter byte advances by four and wraps at 0x40
            CHECK_MSG(sent[i].data[3] == counter, "0x39B counter %02X at %u",
                      (unsigned)sent[i].data[3], (unsigned)i);
            CHECK(sent[i].data[1] == 0x10U && sent[i].data[5] == 0x03U
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
    CHECK(sent_count - before == 108U);

    // the sequence releases the block when it finishes, and can run again
    CHECK(!climate_is_active());
    twai_message_t frame = {0};
    frame.identifier = 0x4A2U;
    CHECK(climate_fwd_hook(&frame, CAN_BUS_1) == FWD_PASSTHROUGH);
}

// The stop direction has no 0x4A2 and no 0x0C7 at all: it is only the session
// bit plus the heartbeat switching to its closing state, which is exactly why
// it is the clearest case of a command that cannot be expressed on M-CAN.
static void suite_stop(void) {
    size_t before = sent_count;
    CHECK(climate_request_start(4) == ESP_OK);
    for (size_t tick = 0; tick < CLIMATE_TEST_TICKS; tick++) {
        climate_tick();
    }

    size_t a2 = 0, c7 = 0, tmu_count = 0, tmu_closing = 0, tmu_idle = 0;
    uint8_t counter = 0U;
    for (size_t i = before; i < sent_count; i++) {
        if (sent[i].id == 0x4A2U) {
            a2++;
        } else if (sent[i].id == 0x0C7U) {
            c7++;
        } else if (sent[i].id == 0x39BU) {
            CHECK(sent[i].data[3] == counter);
            CHECK(sent[i].data[5] == 0x03U);
            CHECK(sent[i].data[0] == 0x00U || sent[i].data[0] == 0x40U);
            if (sent[i].data[0] == 0x40U) {
                tmu_closing++;
            } else {
                tmu_idle++;
            }
            counter = (uint8_t)((counter + 4U) & 0x3FU);
            tmu_count++;
        }
    }
    CHECK(a2 == 0U);
    CHECK(c7 == 0U);
    CHECK(tmu_count == 41U);
    CHECK(tmu_closing == 36U);
    CHECK(tmu_idle == 5U);
    // 3 wake + 3 idle + 41 heartbeat + 41 state + 8 keepalives = 96
    CHECK(sent_count - before == 96U);
    CHECK(!climate_is_active());
}

// Lock and unlock differ only in the apply's state nibble; the request and the
// command class are identical, which is the whole reason both buttons exist.
static void suite_lock_unlock(void) {
    size_t before = sent_count;
    CHECK(climate_request_start(5) == ESP_OK);
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
        } else if (sent[i].id == 0x39BU && (sent[i].data[5] != 0x01U
                                            || sent[i].data[0] != 0x00U)) {
            wrong_class++;
        }
    }
    CHECK(lock_request == 3U);
    CHECK(lock_apply == 3U);
    CHECK(wrong_class == 0U);

    before = sent_count;
    CHECK(climate_request_start(6) == ESP_OK);
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

// The car takes the cabin temperature from 0x39B byte 1 and its 0x2AC mirror,
// raw = 2T - 28, so a request for 22.5 C has to come out as 0x11 on both.
static void suite_setpoint(void) {
    // let the sleeping-car run finish first, so the next request is accepted
    for (size_t tick = 0; tick < CLIMATE_TEST_TICKS && climate_is_active(); tick++) {
        climate_tick();
    }
    CHECK(!climate_is_active());

    size_t before = sent_count;
    CHECK(climate_request_start_temp(0, 225) == ESP_OK);
    for (size_t tick = 0; tick < 26U; tick++) {
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
    CHECK(climate_request_start_temp(0, 100) == ESP_OK);
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
    suite_stop();
    suite_lock_unlock();
    // last: leaves the sequence running, which no later suite would survive
    suite_start_asleep();

    CHECK(wrong_bus == 0);
    // 108 start + 96 stop + 108 lock + 108 unlock, plus the sleeping-car run
    CHECK(sent_count == 420U + asleep_sends);
    suite_setpoint();
    return test_report("test_climate");
}
