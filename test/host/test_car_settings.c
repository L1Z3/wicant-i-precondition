// host-side behavioral test: drives the 0x4C5/0x1F9 charge-limit bridging and
// its injection state machine with a fake clock and a recording can_send.
//
// The firmware module is #included (not linked) so the test can also inspect
// the burst state directly.
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <pthread.h>
#include <sys/wait.h>
#include <unistd.h>
#include "test_support.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "can.h"
#include "config_server.h"
#include "car_settings.h"

// ---- stubs ----
static int cfg_ac = CHARGE_LIMIT_DEFAULT;
static int cfg_dc = CHARGE_LIMIT_DEFAULT;
static int persisted_ac = -1;
static int persisted_dc = -1;

int config_server_get_charge_ac_limit(void) { return cfg_ac; }
int config_server_get_charge_dc_limit(void) { return cfg_dc; }
void config_server_set_charge_ac_limit(uint8_t percent) { persisted_ac = percent; }
void config_server_set_charge_dc_limit(uint8_t percent) { persisted_dc = percent; }

typedef struct {
    can_bus_t bus;
    twai_message_t msg;
} sent_t;
#define SENT_MAX 256
static sent_t sent[SENT_MAX];
static int sent_count = 0;
static bool send_fail = false;

esp_err_t can_send(can_bus_t bus, twai_message_t *message, TickType_t ticks_to_wait) {
    (void)ticks_to_wait;
    // record the attempt even when it fails, so a dropped burst is visible
    if (sent_count < SENT_MAX) {
        sent[sent_count].bus = bus;
        sent[sent_count].msg = *message;
    }
    sent_count++;
    return send_fail ? -1 : ESP_OK;
}

#include "car_settings.c"

// ---- harness ----
static void start(int ac_percent, int dc_percent) {
    cfg_ac = ac_percent;
    cfg_dc = dc_percent;
    persisted_ac = -1;
    persisted_dc = -1;
    sent_count = 0;
    send_fail = false;
    // non-zero so a reply at "t = 0" cannot collide with the never-answered
    // sentinel in the conflict latch
    fake_now = 1000000;
    car_settings_init();
    car_settings_bus_up();
}

static void rx_frame(uint32_t id, const uint8_t *data, uint8_t dlc, can_bus_t bus) {
    twai_message_t f = {0};
    f.identifier = id;
    f.data_length_code = dlc;
    memcpy(f.data, data, dlc < 8 ? dlc : 8);
    car_settings_can_rx_hook(&f, bus);
}

// the head unit's command: D5/D6 are the limits, the rest is the injection
// template the firmware is expected to replay
static const uint8_t CMD_TMPL[8] = {0x11, 0x22, 0x33, 0x44, 0x00, 0x00, 0x77, 0x88};
static const uint8_t CMD_TMPL_B[8] = {0xA1, 0xB2, 0xC3, 0xD4, 0x00, 0x00, 0xE7, 0xF8};
static const uint8_t NO_TMPL[8] = {0};

static void rx_cmd(uint8_t d5, uint8_t d6, can_bus_t bus) {
    uint8_t d[8];
    memcpy(d, CMD_TMPL, sizeof(d));
    d[4] = d5;
    d[5] = d6;
    rx_frame(0x4C5, d, 8, bus);
}

static void rx_reply(uint8_t d3, uint8_t d4, can_bus_t bus) {
    uint8_t d[8] = {0};
    d[2] = d3;
    d[3] = d4;
    rx_frame(0x1F9, d, 8, bus);
}

static void tick1(void) {
    fake_now += 40000;
    car_settings_tick();
}

static void advance_ticks(int n) {
    for (int i = 0; i < n; i++) {
        tick1();
    }
}

static void advance_us(int64_t us) {
    int64_t end = fake_now + us;
    while (fake_now < end) {
        tick1();
    }
}

// tick until one more frame is recorded; returns the number of ticks used
static int tick_until_sent(int max_ticks) {
    int before = sent_count;
    int ticks = 0;
    while (ticks < max_ticks && sent_count == before) {
        tick1();
        ticks++;
    }
    return ticks;
}

// assert one recorded frame is a 0x4C5 on the car bus. For an active frame the
// template bytes are expected verbatim with D5/D6 replaced by the target.
static void check_cmd_frame(int idx, bool passive, const uint8_t tmpl[8],
                            uint8_t raw_ac, uint8_t raw_dc) {
    CHECK_MSG(idx < sent_count, "frame %d missing (only %d sent)", idx, sent_count);
    if (idx >= sent_count) {
        return;
    }
    CHECK(sent[idx].bus == CAN_BUS_0);
    CHECK(sent[idx].msg.identifier == 0x4C5);
    uint8_t expected[8];
    if (passive) {
        memset(expected, 0xFF, sizeof(expected));
        CHECK(sent[idx].msg.data_length_code == 8);
    } else {
        memcpy(expected, tmpl, sizeof(expected));
        expected[4] = raw_ac;
        expected[5] = raw_dc;
        CHECK(sent[idx].msg.data_length_code == 8);
    }
    CHECK_MSG(memcmp(sent[idx].msg.data, expected, sizeof(expected)) == 0,
              "frame %d: got %02X %02X %02X %02X %02X %02X %02X %02X",
              idx,
              sent[idx].msg.data[0], sent[idx].msg.data[1],
              sent[idx].msg.data[2], sent[idx].msg.data[3],
              sent[idx].msg.data[4], sent[idx].msg.data[5],
              sent[idx].msg.data[6], sent[idx].msg.data[7]);
}

// ---- suites ----
static void run_conversions(void) {
    CHECK(charge_limit_percent_to_raw(50) == 0x64);
    CHECK(charge_limit_percent_to_raw(100) == 0xC8);
    CHECK(charge_limit_raw_to_percent(0x64) == 50);
    CHECK(charge_limit_raw_to_percent(0xC8) == 100);
    // 0xFF is the DBC's "off", everything out of range clamps to 0
    CHECK(charge_limit_raw_to_percent(0xFF) == 0);
    CHECK(charge_limit_raw_to_percent(0x00) == 0);
}

static void run_target_set_get(void) {
    start(100, 100);
    CHECK(charge_limit_get_target().ac_percent == 100);
    CHECK(charge_limit_get_target().dc_percent == 100);

    // out-of-range targets are rejected and leave both the RAM copy and the
    // persisted config alone
    CHECK(!charge_limit_set_target((charge_limit_pair_t){.ac_percent = 49, .dc_percent = 100}));
    CHECK(!charge_limit_set_target((charge_limit_pair_t){.ac_percent = 100, .dc_percent = 101}));
    CHECK(charge_limit_get_target().ac_percent == 100);
    CHECK(persisted_ac == -1 && persisted_dc == -1);

    CHECK(charge_limit_set_target((charge_limit_pair_t){.ac_percent = 60, .dc_percent = 70}));
    CHECK(charge_limit_get_target().ac_percent == 60);
    CHECK(charge_limit_get_target().dc_percent == 70);
    // set_target() writes the RAM config; the caller commits it
    CHECK(persisted_ac == 60 && persisted_dc == 70);
}

static void run_reply_validity_and_freshness(void) {
    start(100, 100);

    charge_limit_reply_t reply = {0};
    CHECK(!charge_limit_get_reply(&reply)); // nothing seen yet

    // 0xFF ("off") and a below-minimum value are not configured limits
    rx_reply(0xFF, 0x96, CAR_BUS);
    CHECK(!charge_limit_get_reply(&reply));
    rx_reply(0x62, 0x96, CAR_BUS); // 49%
    CHECK(!charge_limit_get_reply(&reply));
    CHECK(charge_limit_get_reported().ac_percent == 0);

    // too short to carry D3/D4
    uint8_t short_reply[8] = {0, 0, 0x96, 0x96};
    rx_frame(0x1F9, short_reply, 3, CAR_BUS);
    CHECK(!charge_limit_get_reply(&reply));

    // a valid reply feeds the display raw bytes and the reported percents
    rx_reply(0xC8, 0x96, CAR_BUS);
    CHECK(charge_limit_get_reply(&reply));
    CHECK(reply.ac_raw == 0xC8 && reply.dc_raw == 0x96);
    CHECK(reply.age_us == 0);
    CHECK(charge_limit_get_reported().ac_percent == 100);
    CHECK(charge_limit_get_reported().dc_percent == 75);

    // valid until the freshness window expires, age reported either way
    fake_now += 9999000;
    CHECK(charge_limit_get_reply(&reply));
    CHECK(reply.age_us == 9999000);
    fake_now += 1000; // exactly 10 s
    CHECK(!charge_limit_get_reply(&reply));
    CHECK(reply.age_us == 10000000);
    CHECK(reply.ac_raw == 0xC8);
}

static void run_bus_filtering(void) {
    start(100, 100);
    charge_limit_reply_t reply = {0};

#if CAN_BUS_COUNT > 1
    // 0x1F9 is the car's echo: a same-ID frame on the head-unit bus is ignored
    rx_reply(0x96, 0x96, HEAD_UNIT_BUS);
    CHECK(!charge_limit_get_reply(&reply));
    CHECK(charge_limit_get_reported().ac_percent == 0);

    // 0x4C5 is the head unit's command: a same-ID frame on the car bus is
    // ignored, so no template is learned and the quiet timer is not reset
    uint8_t d[8];
    memcpy(d, CMD_TMPL, sizeof(d));
    d[4] = 0x96;
    d[5] = 0x96;
    rx_frame(0x4C5, d, 8, CAR_BUS);
    advance_us(5000000);

    int ticks = tick_until_sent(130); // ~125 ticks = 5 s left of the 10 s
    CHECK(ticks <= 130);
    CHECK(sent_count == 1);
    // no template was learned: replay bytes are zero
    check_cmd_frame(0, false, NO_TMPL, 0xC8, 0xC8);
#else
    // single-bus board: HEAD_UNIT_BUS falls back to the car bus, so both IDs
    // are accepted on CAN_BUS_0
    rx_cmd(0x96, 0x96, HEAD_UNIT_BUS);
    rx_reply(0x96, 0x96, CAR_BUS);
    CHECK(charge_limit_get_reply(&reply));
    CHECK(charge_limit_get_reported().dc_percent == 75);
#endif
}

static void run_quiet_fallback_burst(void) {
    start(100, 100);
    sent_count = 0;

    // head unit asks for 75%/75%; the target is 100%/100%
    rx_cmd(0x96, 0x96, HEAD_UNIT_BUS);

    // nothing while the command is still fresh
    advance_us(8000000);
    CHECK(sent_count == 0);

    // 10 s of silence arms a burst: 3 active frames, one idle tick, 3 passive
    int ticks = tick_until_sent(80);
    CHECK(ticks > 0);
    CHECK(sent_count == 1);
    check_cmd_frame(0, false, CMD_TMPL, 0xC8, 0xC8);

    advance_ticks(2);
    CHECK(sent_count == 3);
    check_cmd_frame(1, false, CMD_TMPL, 0xC8, 0xC8);
    check_cmd_frame(2, false, CMD_TMPL, 0xC8, 0xC8);

    advance_ticks(1); // single idle tick between the phases
    CHECK(sent_count == 3);

    advance_ticks(3); // passive tail, all 0xFF
    CHECK(sent_count == 6);
    check_cmd_frame(3, true, NO_TMPL, 0, 0);
    check_cmd_frame(5, true, NO_TMPL, 0, 0);

    // one burst per quiet episode: no re-arm while the bus stays quiet
    advance_ticks(50);
    CHECK(sent_count == 6);
    CHECK(s_burst_phase == BURST_IDLE);

    // new head-unit traffic re-arms a later quiet episode
    sent_count = 0;
    uint8_t d[8];
    memcpy(d, CMD_TMPL_B, sizeof(d));
    d[4] = 0x64; // 50%
    d[5] = 0x64;
    rx_frame(0x4C5, d, 8, HEAD_UNIT_BUS);
    tick1(); // sees the traffic and clears the quiet latch
    CHECK(sent_count == 0);

    advance_us(10000000);
    ticks = tick_until_sent(80);
    CHECK(sent_count > 0);
    // the template moved on with the newest command frame
    check_cmd_frame(0, false, CMD_TMPL_B, 0xC8, 0xC8);
}

static void run_quiet_fallback_without_template(void) {
    start(100, 100);
    sent_count = 0;

    // no 0x4C5 ever seen: the timeout runs from bus-up and the active frame
    // carries the target with no template to replay
    int ticks = tick_until_sent(300);
    CHECK(ticks > 0 && ticks <= 260);
    CHECK(sent_count == 1);
    check_cmd_frame(0, false, NO_TMPL, 0xC8, 0xC8);
}

static void run_quiet_fallback_ignores_short_command(void) {
    start(100, 100);
    sent_count = 0;

    // a 0x4C5 with neither D5 nor D6 is ignored: no template, and the quiet
    // timer keeps running from bus-up rather than being reset
    advance_us(5000000);
    uint8_t short_cmd[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
    rx_frame(0x4C5, short_cmd, 5, HEAD_UNIT_BUS);
    CHECK(sent_count == 0);

    // if the frame had been adopted this would need ~250 more ticks
    int ticks = tick_until_sent(130);
    CHECK(ticks <= 130);
    CHECK(sent_count == 1);
    check_cmd_frame(0, false, NO_TMPL, 0xC8, 0xC8);
}

static void run_short_template_dlc(void) {
    start(100, 100);
    sent_count = 0;

    // DLC 6 is the minimum that carries D5/D6; the injected frame keeps that
    // DLC instead of padding to 8
    uint8_t cmd[8] = {0x11, 0x22, 0x33, 0x44, 0x96, 0x96, 0, 0};
    rx_frame(0x4C5, cmd, 6, HEAD_UNIT_BUS);

    int ticks = tick_until_sent(300);
    CHECK(ticks > 0);
    CHECK(sent_count == 1);
    CHECK(sent[0].msg.identifier == 0x4C5);
    CHECK(sent[0].msg.data_length_code == 6);
    CHECK(sent[0].msg.data[0] == 0x11 && sent[0].msg.data[3] == 0x44);
    CHECK(sent[0].msg.data[4] == 0xC8 && sent[0].msg.data[5] == 0xC8);
}

static void run_conflict_burst_and_latch(void) {
    start(100, 100);
    sent_count = 0;

    // the car reports 75%/75% against a 100%/100% target
    rx_reply(0x96, 0x96, CAR_BUS);
    CHECK(charge_limit_get_reported().ac_percent == 75);

    // too fresh to answer: back off until the reply is 100 ms old
    advance_ticks(2); // +80 ms
    CHECK(sent_count == 0);
    advance_ticks(1); // +120 ms
    CHECK(sent_count == 1);
    // no 0x4C5 was ever seen, so only D5/D6 are set
    check_cmd_frame(0, false, NO_TMPL, 0xC8, 0xC8);

    advance_ticks(2);
    CHECK(sent_count == 3);
    advance_ticks(1); // idle tick
    CHECK(sent_count == 3);
    advance_ticks(3); // passive tail
    CHECK(sent_count == 6);
    CHECK(s_burst_phase == BURST_IDLE);

    // the car has not moved: an identical echo must not re-arm
    sent_count = 0;
    rx_reply(0x96, 0x96, CAR_BUS);
    advance_ticks(100);
    CHECK(sent_count == 0);

    // the car moving re-arms the burst
    rx_reply(0x82, 0x82, CAR_BUS); // 65%
    sent_count = 0;
    tick_until_sent(10);
    CHECK(sent_count == 1);

    // let that burst finish, then repeat the echo: answered once, no re-arm
    advance_ticks(6);
    sent_count = 0;
    rx_reply(0x82, 0x82, CAR_BUS);
    advance_ticks(100);
    CHECK(sent_count == 0);

    // a changed target re-arms it (charge_limit_set_target leaves the latch)
    CHECK(charge_limit_set_target((charge_limit_pair_t){.ac_percent = 70, .dc_percent = 70}));
    sent_count = 0;
    tick_until_sent(10);
    CHECK(sent_count == 1);
    check_cmd_frame(0, false, NO_TMPL, charge_limit_percent_to_raw(70),
                    charge_limit_percent_to_raw(70));
}

static void run_probe_burst_and_adoption(void) {
    // probe before CAN is up: armed, but nothing goes out
    cfg_ac = 100;
    cfg_dc = 100;
    persisted_ac = -1;
    persisted_dc = -1;
    sent_count = 0;
    send_fail = false;
    fake_now = 1000000;
    car_settings_init();
    car_settings_probe_status();
    advance_ticks(3);
    CHECK(sent_count == 0);

    car_settings_bus_up();
    tick1();
    CHECK(sent_count == 1);
    check_cmd_frame(0, true, NO_TMPL, 0, 0);
    advance_ticks(2);
    CHECK(sent_count == 3); // passive burst only, no active frames
    check_cmd_frame(2, true, NO_TMPL, 0, 0);
    advance_ticks(5);
    CHECK(sent_count == 3);

    // the first valid echo after the probe is adopted as the target and
    // written to the persisted config
    rx_reply(0x96, 0x96, CAR_BUS);
    CHECK(charge_limit_get_target().ac_percent == 75);
    CHECK(charge_limit_get_target().dc_percent == 75);
    CHECK(persisted_ac == 75 && persisted_dc == 75);

    // one-shot: later replies never re-adopt
    rx_reply(0x82, 0x82, CAR_BUS);
    CHECK(charge_limit_get_target().ac_percent == 75);
    CHECK(charge_limit_get_reported().ac_percent == 65);
}

static void run_inject_failure_ends_burst(void) {
    start(100, 100);
    sent_count = 0;

    rx_cmd(0x96, 0x96, HEAD_UNIT_BUS);
    int ticks = tick_until_sent(300);
    CHECK(ticks > 0);
    CHECK(sent_count == 1);
    CHECK(s_burst_phase == BURST_ACTIVE);

    // a failed can_send() drops the rest of the burst instead of retrying
    send_fail = true;
    sent_count = 0;
    tick1();
    CHECK(sent_count == 1); // the attempt is recorded
    CHECK(s_burst_phase == BURST_IDLE);

    // no passive tail, and the quiet latch holds off a fresh burst
    send_fail = false;
    advance_ticks(50);
    CHECK(sent_count == 1);
}

static void run_passive_inject_failure_ends_burst(void) {
    start(100, 100);
    sent_count = 0;

    rx_cmd(0x96, 0x96, HEAD_UNIT_BUS);
    int ticks = tick_until_sent(300);
    CHECK(ticks > 0);
    CHECK(sent_count == 1);

    advance_ticks(2); // remaining active frames
    CHECK(sent_count == 3);
    CHECK(s_burst_phase == BURST_GAP);

    advance_ticks(1); // idle tick: the passive tail is now armed
    CHECK(sent_count == 3);
    CHECK(s_burst_phase == BURST_PASSIVE);

    // a failed passive frame drops the sequence too, rather than retrying (and
    // logging a warning) every 40 ms while the bus is wedged
    send_fail = true;
    sent_count = 0;
    tick1();
    CHECK(sent_count == 1); // the attempt is recorded
    CHECK(s_burst_phase == BURST_IDLE);

    send_fail = false;
    advance_ticks(50);
    CHECK(sent_count == 1); // no retries, no remaining passive frames
}

// ---- suite table ----
// each suite runs in its own forked process: the module's state is all
// process statics
typedef struct {
    const char *name;
    void (*fn)(void);
} suite_t;

static const suite_t suites[] = {
    {"charge limit conversions", run_conversions},
    {"charge limit target set/get", run_target_set_get},
    {"reply validity and freshness", run_reply_validity_and_freshness},
    {"bus filtering", run_bus_filtering},
    {"quiet fallback burst", run_quiet_fallback_burst},
    {"quiet fallback without template", run_quiet_fallback_without_template},
    {"quiet fallback ignores short command", run_quiet_fallback_ignores_short_command},
    {"short command frame template dlc", run_short_template_dlc},
    {"conflict burst and latch", run_conflict_burst_and_latch},
    {"probe burst and adoption", run_probe_burst_and_adoption},
    {"inject failure ends the active burst", run_inject_failure_ends_burst},
    {"inject failure ends the passive burst", run_passive_inject_failure_ends_burst},
};
#define NUM_SUITES (sizeof(suites) / sizeof(suites[0]))

static int run_suite(const suite_t *s) {
    // The Makefile builds this suite for both bus counts and states which one
    // each binary is. Without this, a stub or -D regression could quietly run
    // the two-bus configuration twice and leave the single-bus branches
    // uncovered.
#ifdef EXPECT_CAN_BUS_COUNT
    CHECK(CAN_BUS_COUNT == EXPECT_CAN_BUS_COUNT);
#endif
    s->fn();
    return test_report(s->name);
}

int main(int argc, char **argv) {
    // single-suite run, handy for debugging: test_car_settings <substring>
    if (argc > 1) {
        for (size_t i = 0; i < NUM_SUITES; i++) {
            if (strstr(suites[i].name, argv[1]) != NULL) {
                return run_suite(&suites[i]);
            }
        }
        fprintf(stderr, "no suite matching '%s'\n", argv[1]);
        return 1;
    }
    int rc = 0;
    for (size_t i = 0; i < NUM_SUITES; i++) {
        pid_t child = fork();
        if (child < 0) {
            perror("fork");
            return 1;
        }
        if (child == 0) {
            exit(run_suite(&suites[i]));
        }
        int status = 0;
        waitpid(child, &status, 0);
        if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
            rc = 1;
        }
    }
    return rc;
}
