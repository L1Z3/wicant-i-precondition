// host-side behavioral test: drives the precondition state machine with a
// fake clock and a recording can_send
//
// The firmware modules are #included (not linked) so the test can inspect
// state-machine and persistence internals directly.
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>
#include <sys/wait.h>
#include <unistd.h>
#include "test_support.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "can.h"
#include "config_server.h"
#include "track_popup.h"

// ---- stubs ----
typedef struct { can_bus_t bus; twai_message_t msg; } sent_t;
static sent_t sent[4096];
static int sent_count = 0;
esp_err_t can_send(can_bus_t bus, twai_message_t *message, TickType_t ticks_to_wait) {
    (void)ticks_to_wait;
    if (sent_count < (int)(sizeof(sent) / sizeof(sent[0]))) {
        sent[sent_count].bus = bus;
        sent[sent_count].msg = *message;
    }
    sent_count++;
    return 0;
}

typedef struct { unsigned char data[64]; int has; UBaseType_t size; } fake_queue_t;
QueueHandle_t xQueueCreate(UBaseType_t len, UBaseType_t item_size) {
    (void)len;
    fake_queue_t *q = calloc(1, sizeof(*q));
    q->size = item_size;
    return q;
}
BaseType_t xQueueOverwrite(QueueHandle_t qh, const void *item) {
    fake_queue_t *q = qh;
    memcpy(q->data, item, q->size);
    q->has = 1;
    return 1;
}
BaseType_t xQueuePeek(QueueHandle_t qh, void *item, TickType_t w) {
    (void)w;
    fake_queue_t *q = qh;
    if (!q->has) return 0;
    memcpy(item, q->data, q->size);
    return 1;
}
BaseType_t xQueueReceive(QueueHandle_t qh, void *item, TickType_t w) {
    (void)w;
    fake_queue_t *q = qh;
    if (!q->has) return 0;
    memcpy(item, q->data, q->size);
    q->has = 0;
    return 1;
}

static int8_t cfg_button = SW_STAR;
static int8_t cfg_mode = ONCE;
static int8_t cfg_press = PRESS_SHORT;
int8_t config_server_precon_button(void) { return cfg_button; }
int8_t config_server_precon_mode(void) { return cfg_mode; }
int8_t config_server_precon_press(void) { return cfg_press; }

// Track popup behavior has its own suite. These stubs keep this test focused
// on precondition behavior while still exercising the global-hook delegation.
static int popup_show_count = 0;
static char popup_text[128];
void track_popup_init(void) {}
void track_popup_tick(void) {}
void track_popup_rx(const twai_message_t *msg, can_bus_t rx_bus) {
    (void)msg;
    (void)rx_bus;
}
fwd_result_t track_popup_fwd(twai_message_t *msg, can_bus_t fwd_bus) {
    (void)msg;
    (void)fwd_bus;
    return FWD_PASSTHROUGH;
}
bool track_popup_show(const char *utf8_text) {
    popup_show_count++;
    snprintf(popup_text, sizeof(popup_text), "%s", utf8_text);
    return true;
}

#include "persistent_settings.c"
#include "precondition.c"

// ---- harness ----
static void expect_state(const char *name) {
    CHECK_MSG(strcmp(precon_sm.current->name, name) == 0,
              "expected state %s, got %s", name, precon_sm.current->name);
}

static void rx_frame(uint32_t id, const uint8_t d[8], can_bus_t bus) {
    twai_message_t f = {0};
    f.identifier = id;
    f.data_length_code = 8;
    memcpy(f.data, d, 8);
    precondition_can_rx_hook(&f, bus);
}

static void press(void)   { uint8_t d[8] = {0}; d[5] = 0x10; rx_frame(0x448, d, CAN_BUS_0); }
static void release(void) { uint8_t d[8] = {0}; rx_frame(0x448, d, CAN_BUS_0); }
static void toggle(void)  { press(); fake_now += 100000; release(); }
static void car_status(uint8_t b, can_bus_t bus) { uint8_t d[8] = {0}; d[1] = b; rx_frame(0x2AD, d, bus); }
static void car_power(bool ready) { uint8_t d[8] = {0}; d[0] = ready ? 0x04 : 0x00; rx_frame(0x038, d, CAN_BUS_0); }
static void battery_temperature(int8_t min_c, int8_t max_c) {
    uint8_t d[8] = {(uint8_t)min_c, (uint8_t)max_c};
    rx_frame(0x152, d, CAN_BUS_0);
}
static void battery_soc(uint8_t raw) {
    uint8_t d[8] = {[7] = raw};
    rx_frame(0x2FC, d, CAN_BUS_0);
}

// Model the two firmware workers in deterministic order: the timing task runs
// the state machine, then the lower-priority persistence task gets CPU time.
static void tick1(void) {
    fake_now += 40000;
    precondition_tick();
    if (cfg_mode == PERSISTENT) {
        (void)flush_pending_settings();
    }
}
static void advance_us(int64_t us) {
    int64_t end = fake_now + us;
    while (fake_now < end) tick1();
}

static void advance_until_state(const char *name, int64_t max_us) {
    int64_t end = fake_now + max_us;
    while (fake_now < end && strcmp(precon_sm.current->name, name) != 0) tick1();
    expect_state(name);
}

static fwd_result_t fwd(uint32_t id, can_bus_t bus, twai_message_t *out) {
    twai_message_t f = {0};
    f.identifier = id;
    f.data_length_code = 8;
    fwd_result_t r = precondition_fwd_hook(&f, bus);
    if (out) *out = f;
    return r;
}

static void check_start_burst_msgs(int base) {
    bool recorded = base >= 0
                    && base + 6 <= sent_count
                    && base + 6 <= (int)(sizeof(sent) / sizeof(sent[0]));
    CHECK(recorded);
    if (!recorded) {
        return;
    }
    for (int i = 0; i < 6; i++) {
        uint8_t expected[8] = {0};
        if (i < 3) {
            expected[3] = 0x40;
            expected[4] = 0x03;
        } else {
            expected[3] = 0xE0;
            expected[4] = 0x07;
        }
        CHECK(sent[base + i].bus == CAR_BUS);
        CHECK(sent[base + i].msg.identifier == 0x0C7);
        CHECK(sent[base + i].msg.data_length_code == 8);
        CHECK(memcmp(sent[base + i].msg.data, expected, sizeof(expected)) == 0);
    }
}

static void check_stop_burst_msgs(int base) {
    bool recorded = base >= 0
                    && base + 6 <= sent_count
                    && base + 6 <= (int)(sizeof(sent) / sizeof(sent[0]));
    CHECK(recorded);
    if (!recorded) {
        return;
    }
    for (int i = 0; i < 6; i++) {
        uint8_t expected[8] = {0};
        if (i >= 3) {
            expected[3] = 0xE0;
            expected[4] = 0x07;
        }
        CHECK(sent[base + i].bus == CAR_BUS);
        CHECK(sent[base + i].msg.identifier == 0x0C7);
        CHECK(sent[base + i].msg.data_length_code == 8);
        CHECK(memcmp(sent[base + i].msg.data, expected, sizeof(expected)) == 0);
    }
}

// snapshot the web UI display state: push_precondition_state() derives it from
// the machine's current leaf, so call it directly (the test includes the
// source) rather than waiting for a tick, then read the queue like the UI does
static precondition_state_t precondition_display(void) {
    precondition_state_t s;
    push_precondition_state();
    if (!precondition_get_state(&s)) {
        s = (precondition_state_t){0};
    }
    return s;
}

static void run_short_press(void) {
    precondition_init();
    expect_state("idle");
    CHECK(!precondition_display().starting);
    CHECK(!precondition_display().active);

    // --- platform without status frames: no retries, fake-active at 70s ---
    sent_count = 0;
    toggle();
    expect_state("start-burst");
    CHECK(popup_show_count == 1);
    CHECK(strcmp(popup_text, "Once: starting") == 0);
    for (int i = 0; i < 6; i++) tick1();
    CHECK(sent_count == 6);
    check_start_burst_msgs(0);
    expect_state("wait-starting");
    // display: BLUE_4 flag, no retry digit (unknown platform)
    twai_message_t m;
    CHECK(fwd(0x4E8, CAN_BUS_0, &m) == FWD_MODIFIED);
    CHECK((m.data[6] & 0x0F) == 0x4);         // FLAG_BLUE_4
    CHECK(fwd(0x0C7, CAN_BUS_0, NULL) == FWD_BLOCK);
    CHECK(fwd(0x4ED, CAN_BUS_0, &m) == FWD_MODIFIED);
    CHECK(m.data[5] == 0x10 && m.data[6] == 0xA0 && m.data[7] == 0x00);
    CHECK(fwd(0x0C7, CAN_BUS_1, NULL) == FWD_PASSTHROUGH);  // head unit bus untouched
    CHECK(fwd(0x4E8, CAN_BUS_1, NULL) == FWD_PASSTHROUGH);
    CHECK(fwd(0x4CC, CAN_BUS_1, NULL) == FWD_PASSTHROUGH);
    sent_count = 0;
    advance_us(71000000);
    expect_state("active");
    CHECK(sent_count == 0);                    // no retries without status frames
    CHECK(precondition_display().active);
    CHECK(!precondition_display().starting);
    CHECK(fwd(0x4E8, CAN_BUS_0, NULL) == FWD_PASSTHROUGH);  // display off in active
    CHECK(fwd(0x0C7, CAN_BUS_0, NULL) == FWD_BLOCK);        // still blocking
    // stop without status frames: burst then straight to idle, no display
    fake_now += 2000000;
    toggle();
    expect_state("stop-burst");
    CHECK(popup_show_count == 2);
    CHECK(strcmp(popup_text, "Once: stopping") == 0);
    CHECK(fwd(0x4E8, CAN_BUS_0, NULL) == FWD_PASSTHROUGH);
    sent_count = 0;
    for (int i = 0; i < 6; i++) tick1();
    CHECK(sent_count == 6);
    check_stop_burst_msgs(0);
    expect_state("idle");
    CHECK(!precondition_display().active);
    CHECK(!precondition_display().starting);

    // --- happy path with status frames ---
    toggle();
    expect_state("start-burst");
    car_status(0x01, CAN_BUS_1);               // head unit bus: must be ignored
    CHECK(platform.precon_status == PRECON_STATUS_UNKNOWN);
    car_status(0x00, CAN_BUS_0);               // unrecognized status remains unknown
    CHECK(platform.precon_status == PRECON_STATUS_UNKNOWN);
    car_status(0x01, CAN_BUS_0);               // idle is now the latest car status
    CHECK(platform.precon_status == PRECON_STATUS_IDLE);
    expect_state("start-burst");
    for (int i = 0; i < 6; i++) tick1();
    expect_state("wait-starting");
    CHECK(precondition_display().starting);
    CHECK(fwd(0x4E8, CAN_BUS_0, &m) == FWD_MODIFIED);
    CHECK((m.data[6] & 0x0F) == 0x1);          // FLAG_BLUE_1 before starting confirm
    car_status(0x45, CAN_BUS_0);               // EV6-style "starting"
    expect_state("wait-started");
    CHECK(precondition_display().starting);
    CHECK(!precondition_display().active);
    CHECK(fwd(0x4E8, CAN_BUS_0, &m) == FWD_MODIFIED);
    CHECK((m.data[6] & 0x0F) == 0x0);          // FLAG_DESTINATION after starting confirm
    car_status(0x55, CAN_BUS_0);               // EV6-style "started"
    expect_state("active");
    CHECK(precondition_display().active);
    CHECK(!precondition_display().starting);
    // Once-mode downgrade stays within the original request and retains its
    // retry budget.
    requested.retries = 2;
    car_status(0x05, CAN_BUS_0);
    expect_state("wait-started");
    CHECK(requested.kind == ATTEMPT_MANUAL);
    CHECK(requested.retries == 2);
    CHECK(precondition_display().starting);
    CHECK(!precondition_display().active);
    car_status(0x15, CAN_BUS_0);
    expect_state("active");
    CHECK(precondition_display().active);
    // "complete" in ONCE mode -> real stop
    car_status(0x01, CAN_BUS_0);
    expect_state("stop-burst");
    CHECK(strcmp(popup_text, "Once: stopping") == 0);
    CHECK(!precondition_display().active);
    CHECK(!precondition_display().starting);
    car_status(0x01, CAN_BUS_0);               // stop confirm ignored during burst
    expect_state("stop-burst");
    CHECK(fwd(0x4E8, CAN_BUS_1, NULL) == FWD_PASSTHROUGH);
    CHECK(fwd(0x4CC, CAN_BUS_1, NULL) == FWD_PASSTHROUGH);
    CHECK(fwd(0x4E8, CAN_BUS_0, &m) == FWD_MODIFIED);  // stop retry display (retries=0)
    CHECK((m.data[0] & 0x0F) == 0x3);          // DIST_UNIT_FT on first stop attempt
    for (int i = 0; i < 6; i++) tick1();
    expect_state("wait-stopped");
    car_status(0x01, CAN_BUS_0);
    expect_state("idle");
    CHECK(!precondition_display().active);
    CHECK(!precondition_display().starting);

    // --- debounce ---
    toggle();
    expect_state("start-burst");
    toggle();                                   // within 1s: debounced away
    CHECK(sm_in(&precon_sm, &S_REQUESTED));
    advance_us(1200000);
    expect_state("wait-starting");
    CHECK(precondition_display().starting);
    toggle();                                   // past debounce: stop
    expect_state("stop-burst");
    CHECK(!precondition_display().starting);
    toggle();                                   // toggle during stopping restarts
    expect_state("start-burst");
    advance_us(1200000);
    toggle();
    for (int i = 0; i < 6; i++) tick1();
    expect_state("wait-stopped");
    car_status(0x01, CAN_BUS_0);
    expect_state("idle");

    // --- start retries, then give-up with silent stop ---
    toggle();
    sent_count = 0;
    for (int i = 0; i < 6; i++) tick1();
    expect_state("wait-starting");
    CHECK(requested.retries == 0);
    CHECK(precondition_display().starting);
    for (int r = 1; r <= 4; r++) {
        advance_until_state("start-burst", 11000000);   // >10s: retry
        CHECK(requested.retries == r);
        for (int i = 0; i < 6; i++) tick1();
        expect_state("wait-starting");
        CHECK(precondition_display().starting);
    }
    CHECK(sent_count == 5 * 6);
    CHECK(fwd(0x4E8, CAN_BUS_0, &m) == FWD_MODIFIED);
    CHECK((m.data[0] >> 4) == 4);               // retry count in tenths digit
    // retries exhausted: give up with a silent stop
    int base = sent_count;
    advance_until_state("stop-burst", 11000000);
    CHECK(stopping.retries == 4);
    CHECK(!precondition_display().starting);
    CHECK(!precondition_display().active);
    CHECK(fwd(0x4E8, CAN_BUS_0, NULL) == FWD_PASSTHROUGH);  // silent stop: no display
    for (int i = 0; i < 6; i++) tick1();
    expect_state("wait-stopped");
    CHECK(sent_count == base + 6);
    check_stop_burst_msgs(base);
    advance_us(10100000);                       // no stop retries in the give-up case
    expect_state("idle");
    CHECK(sent_count == base + 6);
    CHECK(!precondition_display().starting);

    // --- wait-started timeout retries at 70s, confirm mid-burst ---
    toggle();
    for (int i = 0; i < 6; i++) tick1();
    car_status(0x05, CAN_BUS_0);
    expect_state("wait-started");
    CHECK(precondition_display().starting);
    advance_us(10100000);
    expect_state("wait-started");               // 10s retry does NOT apply here
    advance_until_state("start-burst", 61000000);  // ...but 70s does
    CHECK(requested.retries == 1);
    sent_count = 0;
    for (int i = 0; i < 3; i++) tick1();        // mid-burst
    expect_state("start-burst");
    CHECK(!precondition_display().starting);  // timeout cleared the wait flag
    car_status(0x15, CAN_BUS_0);                // started confirm mid-burst
    expect_state("start-burst");                // burst still runs to completion
    CHECK(fwd(0x4E8, CAN_BUS_0, NULL) == FWD_PASSTHROUGH);  // but display is off
    for (int i = 0; i < 3; i++) tick1();
    CHECK(sent_count == 6);                     // full burst sent despite confirm
    expect_state("active");
    CHECK(precondition_display().active);
    CHECK(!precondition_display().starting);
    // stop retries
    fake_now += 2000000;
    toggle();
    for (int i = 0; i < 6; i++) tick1();
    expect_state("wait-stopped");
    CHECK(stopping.retries == 0);
    advance_until_state("stop-burst", 11000000);
    CHECK(stopping.retries == 1);
    for (int i = 0; i < 6; i++) tick1();
    expect_state("wait-stopped");
    car_status(0x01, CAN_BUS_0);
    expect_state("idle");
    CHECK(!precondition_display().active);

    // --- the car turning off silently abandons an attempt (all modes) ---
    car_power(true);
    expect_state("idle");                      // ready edge does nothing in once mode
    toggle();
    for (int i = 0; i < 6; i++) tick1();
    expect_state("wait-starting");
    CHECK(precondition_display().starting);
    sent_count = 0;
    car_power(false);
    expect_state("idle");
    CHECK(!precondition_display().starting);
    CHECK(!idle.continuous_disabled_by_car_off);
    CHECK(sent_count == 0);                    // no stop burst: nothing left to stop
    CHECK(fwd(0x0C7, CAN_BUS_0, NULL) == FWD_PASSTHROUGH);

    // --- car off during an active stop abandons the stop ---
    car_power(true);
    toggle();
    for (int i = 0; i < 6; i++) tick1();
    expect_state("wait-starting");
    fake_now += 2000000;
    toggle();
    expect_state("stop-burst");
    sent_count = 0;
    car_power(false);
    expect_state("idle");
    advance_us(15000000);
    CHECK(sent_count == 0);                    // no burst continuation, no retries
}

static void run_long_press(void) {
    precondition_init();
    expect_state("idle");
    // quick press+release in long mode: nothing happens
    toggle();
    advance_us(2000000);
    expect_state("idle");
    // hold crossing the threshold fires once, without release
    press();
    advance_us(500000);
    expect_state("idle");
    sent_count = 0;
    advance_us(600000);                         // crosses 1s: fires exactly once
    CHECK(sm_in(&precon_sm, &S_REQUESTED));
    CHECK(sent_count > 0);                      // burst began on the fire tick
    advance_us(2000000);                        // keep holding: no second fire
    CHECK(sm_in(&precon_sm, &S_REQUESTED));     // (a re-fire would stop it)
    release();
    CHECK(sm_in(&precon_sm, &S_REQUESTED));     // release in long mode is a no-op
}

static void run_battery_temperature_cutoff(void) {
    precondition_init();
    expect_state("idle");
    CHECK(precon_blockers == PRECONDITION_BLOCK_NONE);

    // The cutoff is inclusive. A manual attempt displays an error and takes
    // the stop path without emitting any start frames.
    battery_temperature(21, 24);
    CHECK(precon_blockers == PRECONDITION_BLOCK_BATTERY_WARM);
    sent_count = 0;
    toggle();
    expect_state("stop-burst");
    CHECK(popup_show_count == 1);
    CHECK(strcmp(popup_text, "Once: temp too high: 21°C ≥ 21°C") == 0);
    CHECK(sent_count == 0);
    for (int i = 0; i < 6; i++) tick1();
    expect_state("idle");
    CHECK(sent_count == 6);
    check_stop_burst_msgs(0);

    // A reading just below the cutoff still allows the normal start sequence.
    battery_temperature(20, 24);
    CHECK(precon_blockers == PRECONDITION_BLOCK_NONE);
    sent_count = 0;
    toggle();
    expect_state("start-burst");
    tick1();
    CHECK(sent_count == 1);
    CHECK(sent[0].msg.data[3] == 0x40 && sent[0].msg.data[4] == 0x03);
    CHECK(popup_show_count == 2);
    CHECK(strcmp(popup_text, "Once: starting") == 0);

    // A newly hot reading also aborts an in-flight manual attempt before a
    // status confirmation can move it into ACTIVE.
    for (int i = 0; i < 5; i++) tick1();
    expect_state("wait-starting");
    battery_temperature(21, 24);
    CHECK(precon_blockers == PRECONDITION_BLOCK_BATTERY_WARM);
    tick1();
    expect_state("stop-burst");
    CHECK(popup_show_count == 3);
    CHECK(strcmp(popup_text, "Once: temp too high: 21°C ≥ 21°C") == 0);
}

static void run_battery_soc(void) {
    precondition_init();
    precondition_soc_t soc;
    twai_message_t frame = {
        .identifier = BATTERY_SOC_FRAME_ID,
        .data_length_code = BATTERY_SOC_DATA_LENGTH,
        .data = {[BATTERY_SOC_INDEX] = 127},
    };

    CHECK(!precondition_get_battery_soc(&soc));
    precondition_can_rx_hook(&frame, CAN_BUS_1);
    CHECK(!precondition_get_battery_soc(&soc));
    frame.data_length_code--;
    precondition_can_rx_hook(&frame, CAN_BUS_0);
    CHECK(!precondition_get_battery_soc(&soc));

    frame.data_length_code++;
    fake_now = 123456;
    precondition_can_rx_hook(&frame, CAN_BUS_0);
    CHECK(precondition_get_battery_soc(&soc));
    CHECK(soc.raw == 127);
    CHECK(soc.raw * BATTERY_SOC_SCALE == 63.5f);
    CHECK(soc.updated_at_us == fake_now);
    CHECK(precon_blockers == PRECONDITION_BLOCK_NONE);

    // The low-SoC cutoff is exclusive. It takes display priority over a warm
    // battery and sends the attempt down the stop path without start frames.
    battery_temperature(21, 24);
    battery_soc(39);
    CHECK(precon_blockers
          == (PRECONDITION_BLOCK_BATTERY_WARM | PRECONDITION_BLOCK_BATTERY_LOW_SOC));
    sent_count = 0;
    toggle();
    expect_state("stop-burst");
    CHECK(popup_show_count == 1);
    CHECK(strcmp(popup_text, "Once: SoC too low: 19.5% < 20%") == 0);
    CHECK(sent_count == 0);
    for (int i = 0; i < 6; i++) tick1();
    expect_state("idle");
    CHECK(sent_count == 6);
    check_stop_burst_msgs(0);

    // Exactly 20% clears the blocker and permits the normal start sequence.
    battery_temperature(20, 24);
    battery_soc(40);
    CHECK(precon_blockers == PRECONDITION_BLOCK_NONE);
    sent_count = 0;
    toggle();
    expect_state("start-burst");
    tick1();
    CHECK(sent_count == 1);
    CHECK(sent[0].msg.data[3] == 0x40 && sent[0].msg.data[4] == 0x03);
    CHECK(popup_show_count == 2);
    CHECK(strcmp(popup_text, "Once: starting") == 0);

    // A newly low reading aborts an in-flight attempt too.
    for (int i = 0; i < 5; i++) tick1();
    expect_state("wait-starting");
    battery_soc(39);
    CHECK(precon_blockers == PRECONDITION_BLOCK_BATTERY_LOW_SOC);
    tick1();
    expect_state("stop-burst");
    CHECK(popup_show_count == 3);
    CHECK(strcmp(popup_text, "Once: SoC too low: 19.5% < 20%") == 0);
}

static void run_automatic_temperature_cutoff(void) {
    precondition_init();
    battery_temperature(20, 24);
    car_power(true);
    toggle();
    for (int i = 0; i < 6; i++) tick1();
    car_status(0x15, CAN_BUS_0);
    expect_state("active");

    // A periodic attempt at the cutoff is dropped silently and leaves the
    // repeating-mode latch enabled.
    int popup_count_before_periodic = popup_show_count;
    battery_temperature(21, 24);
    car_status(0x01, CAN_BUS_0);
    sent_count = 0;
    advance_us(REPEATING_MODE_RETRY_INTERVAL_US + 80000);
    expect_state("managed");
    CHECK(sent_count == 0);
    CHECK(popup_show_count == popup_count_before_periodic);
    CHECK(repeating_mode_enabled());
}

static void run_automatic_soc_cutoff(void) {
    precondition_init();
    battery_soc(40);
    car_power(true);
    toggle();
    for (int i = 0; i < 6; i++) tick1();
    car_status(0x15, CAN_BUS_0);
    expect_state("active");

    // The falling edge is announced immediately, even if the active context
    // came from a silent BMU restart. Repeated low frames and periodic attempts
    // remain silent.
    car_status(0x05, CAN_BUS_0);
    car_status(0x15, CAN_BUS_0);
    CHECK(requested.kind == ATTEMPT_BMU_RESTART);
    int popup_count_before_low_soc = popup_show_count;
    battery_soc(39);
    CHECK(popup_show_count == popup_count_before_low_soc + 1);
    CHECK(strcmp(popup_text, "Continuous: resuming when SoC ≥ 20%") == 0);
    battery_soc(39);
    CHECK(popup_show_count == popup_count_before_low_soc + 1);
    car_status(0x01, CAN_BUS_0);
    sent_count = 0;
    advance_us(REPEATING_MODE_RETRY_INTERVAL_US + 80000);
    expect_state("managed");
    CHECK(sent_count == 0);
    CHECK(popup_show_count == popup_count_before_low_soc + 1);
    CHECK(repeating_mode_enabled());

    // Recovery rearms the edge notice, including while already MANAGED.
    battery_soc(40);
    battery_soc(39);
    CHECK(popup_show_count == popup_count_before_low_soc + 2);
    CHECK(strcmp(popup_text, "Continuous: resuming when SoC ≥ 20%") == 0);
}

static void run_repeating_soc_cutoff_message(void) {
    precondition_init();
    battery_temperature(21, 24);
    battery_soc(39);
    CHECK(precon_blockers
          == (PRECONDITION_BLOCK_BATTERY_WARM | PRECONDITION_BLOCK_BATTERY_LOW_SOC));

    sent_count = 0;
    toggle();
    expect_state("managed");
    CHECK(sent_count == 0);
    CHECK(repeating_mode_enabled());
    CHECK(popup_show_count == 1);

    const char *expected = cfg_mode == PERSISTENT
                         ? "Persistent: resuming when SoC ≥ 20%"
                         : "Continuous: resuming when SoC ≥ 20%";
    CHECK(strcmp(popup_text, expected) == 0);

    if (cfg_mode == PERSISTENT) {
        // A later car restart is another user-visible resume attempt.
        car_power(true);
        expect_state("car-start-delay");
        advance_us(PRECONDITION_CAR_START_DELAY_US - 40000);
        tick1();
        expect_state("managed");
        CHECK(requested.kind == ATTEMPT_CAR_START);
        CHECK(sent_count == 0);
        CHECK(popup_show_count == 2);
        CHECK(strcmp(popup_text, expected) == 0);
    }
}

static void run_repeating_temperature_notice(void) {
    precondition_init();
    battery_temperature(23, 24);
    battery_soc(40);

    sent_count = 0;
    toggle();
    expect_state("managed");
    CHECK(sent_count == 0);
    CHECK(repeating_mode_enabled());
    CHECK(popup_show_count == 1);

    const char *expected = cfg_mode == PERSISTENT
                         ? "Persistent: maintaining 21°C (23°C)"
                         : "Continuous: maintaining 21°C (23°C)";
    CHECK(strcmp(popup_text, expected) == 0);

    if (cfg_mode == PERSISTENT) {
        car_power(true);
        expect_state("car-start-delay");
        advance_us(PRECONDITION_CAR_START_DELAY_US - 40000);
        tick1();
        expect_state("managed");
        CHECK(requested.kind == ATTEMPT_CAR_START);
        CHECK(sent_count == 0);
        CHECK(popup_show_count == 2);
        CHECK(strcmp(popup_text, expected) == 0);
    }
}

static void run_once_stopping_notices(void) {
    precondition_init();
    battery_temperature(20, 24);
    battery_soc(40);

    // A temperature reading received before the idle status gives the
    // best-effort reached-target reason.
    toggle();
    for (int i = 0; i < 6; i++) tick1();
    car_status(0x15, CAN_BUS_0);
    expect_state("active");
    CHECK(requested.was_active);
    int popup_count_before_limit = popup_show_count;
    battery_temperature(21, 24);
    CHECK(popup_show_count == popup_count_before_limit);
    car_status(0x01, CAN_BUS_0);
    expect_state("stop-burst");
    CHECK(stopping.reason == STOP_REASON_TEMPERATURE_REACHED);
    CHECK(popup_show_count == popup_count_before_limit + 1);
    CHECK(strcmp(popup_text, "Once: stopping (reached 21°C)") == 0);
    for (int i = 0; i < 6; i++) tick1();
    car_status(0x01, CAN_BUS_0);
    expect_state("idle");

    // SoC wins when both known limits are active after the session was active.
    battery_temperature(20, 24);
    battery_soc(40);
    toggle();
    for (int i = 0; i < 6; i++) tick1();
    car_status(0x15, CAN_BUS_0);
    expect_state("active");
    popup_count_before_limit = popup_show_count;
    battery_temperature(21, 24);
    battery_soc(39);
    CHECK(popup_show_count == popup_count_before_limit);
    car_status(0x01, CAN_BUS_0);
    expect_state("stop-burst");
    CHECK(stopping.reason == STOP_REASON_LOW_SOC);
    CHECK(popup_show_count == popup_count_before_limit + 1);
    CHECK(strcmp(popup_text, "Once: stopping (<20% SoC)") == 0);
    for (int i = 0; i < 6; i++) tick1();
    car_status(0x01, CAN_BUS_0);
    expect_state("idle");

    // If idle arrives before the measurement that explains it, fall back to
    // the generic notice and do not replace it with a late temperature frame.
    battery_temperature(20, 24);
    battery_soc(40);
    toggle();
    for (int i = 0; i < 6; i++) tick1();
    car_status(0x15, CAN_BUS_0);
    expect_state("active");
    int popup_count_before_idle = popup_show_count;
    car_status(0x01, CAN_BUS_0);
    expect_state("stop-burst");
    CHECK(stopping.reason == STOP_REASON_UNEXPECTED_IDLE);
    CHECK(popup_show_count == popup_count_before_idle + 1);
    CHECK(strcmp(popup_text, "Once: stopping") == 0);
    battery_temperature(21, 24);
    CHECK(popup_show_count == popup_count_before_idle + 1);
    CHECK(strcmp(popup_text, "Once: stopping") == 0);
}

typedef enum {
    PRECON_CONCURRENT_TICK,
    PRECON_CONCURRENT_RX,
    PRECON_CONCURRENT_FWD,
} precon_concurrent_op_t;

typedef struct {
    precon_concurrent_op_t op;
    pthread_barrier_t *start;
    unsigned int iterations;
} precon_concurrent_worker_t;

static void *precon_concurrent_worker(void *arg) {
    precon_concurrent_worker_t *worker = arg;
    pthread_barrier_wait(worker->start);
    for (unsigned int i = 0; i < worker->iterations; i++) {
        twai_message_t frame = { .data_length_code = 8 };
        switch (worker->op) {
            case PRECON_CONCURRENT_TICK:
                precondition_tick();
                break;
            case PRECON_CONCURRENT_RX:
                frame.identifier = 0x448;
                frame.data[5] = (i & 1U) ? 0x00 : 0x10;
                precondition_can_rx_hook(&frame, CAN_BUS_0);
                break;
            case PRECON_CONCURRENT_FWD:
                frame.identifier = 0x4E8;
                precondition_fwd_hook(&frame, CAN_BUS_0);
                break;
        }
    }
    return NULL;
}

static void run_concurrent_dispatch(void) {
    enum { WORKER_COUNT = 3, ITERATIONS = 1000 };
    precondition_init();
    fake_now = 1000000;

    pthread_barrier_t start;
    CHECK(pthread_barrier_init(&start, NULL, WORKER_COUNT) == 0);
    pthread_t threads[WORKER_COUNT];
    precon_concurrent_worker_t workers[WORKER_COUNT];
    for (int i = 0; i < WORKER_COUNT; i++) {
        workers[i] = (precon_concurrent_worker_t){
            .op = (precon_concurrent_op_t)i,
            .start = &start,
            .iterations = ITERATIONS,
        };
        CHECK(pthread_create(&threads[i], NULL, precon_concurrent_worker, &workers[i]) == 0);
    }
    for (int i = 0; i < WORKER_COUNT; i++) {
        CHECK(pthread_join(threads[i], NULL) == 0);
    }
    CHECK(pthread_barrier_destroy(&start) == 0);
    expect_state("idle");
    CHECK(sent_count == 0);
}

static void run_continuous(void) {
    precondition_init();
    expect_state("idle");
    battery_temperature(-5, 2);
    car_power(true);
    expect_state("idle");                       // ready edge alone starts nothing

    // manual start shows the countdown and becomes ACTIVE once confirmed
    toggle();
    expect_state("start-burst");
    CHECK(strcmp(popup_text,
                 "Continuous: maintaining 21°C (-5°C)") == 0);
    twai_message_t m;
    CHECK(fwd(0x4E8, CAN_BUS_0, &m) == FWD_MODIFIED);
    for (int i = 0; i < 6; i++) tick1();
    expect_state("wait-starting");
    CHECK(precondition_display().starting);
    car_status(0x05, CAN_BUS_0);
    expect_state("wait-started");
    CHECK(precondition_display().starting);
    car_status(0x15, CAN_BUS_0);
    expect_state("active");
    CHECK(precondition_display().active);
    CHECK(!precondition_display().starting);
    // MITM stays on while preconditioning is active; the display does not
    CHECK(fwd(0x0C7, CAN_BUS_0, NULL) == FWD_BLOCK);
    CHECK(fwd(0x4ED, CAN_BUS_0, &m) == FWD_MODIFIED);
    CHECK(m.data[5] == 0x10 && m.data[6] == 0xA0 && m.data[7] == 0x00);
    CHECK(fwd(0x4E8, CAN_BUS_0, NULL) == FWD_PASSTHROUGH);

    // A downgrade while a repeating session is active is a fresh, silent BMU
    // restart rather than a continuation of the old attempt.
    requested.retries = 3;
    int sent_before_bmu_restart = sent_count;
    car_status(0x05, CAN_BUS_0);
    expect_state("wait-started");
    CHECK(requested.kind == ATTEMPT_BMU_RESTART);
    CHECK(requested.retries == 0);
    CHECK(requested.last_attempt_ts == fake_now);
    CHECK(fwd(0x4E8, CAN_BUS_0, NULL) == FWD_PASSTHROUGH);
    CHECK(sent_count == sent_before_bmu_restart);
    car_status(0x15, CAN_BUS_0);
    expect_state("active");

    // while the BMU reports running, no re-requests ever fire
    sent_count = 0;
    advance_us(310000000LL);
    car_status(0x15, CAN_BUS_0);
    advance_us(310000000LL);
    CHECK(sent_count == 0);
    expect_state("active");

    // BMU stops: enter MANAGED and arm the 5-minute re-nudge on this idle edge
    car_status(0x01, CAN_BUS_0);
    expect_state("managed");
    CHECK(managed.nudge_base_ts == fake_now);
    CHECK(!precondition_display().active);             // idle while waiting to re-nudge
    advance_us(295000000LL);                    // just under 5 minutes
    CHECK(sent_count == 0);
    expect_state("managed");
    advance_until_state("start-burst", 10000000LL);
    CHECK(requested.kind == ATTEMPT_PERIODIC);
    CHECK(!precondition_display().starting);  // start-burst is not a wait state
    CHECK(fwd(0x4E8, CAN_BUS_0, NULL) == FWD_PASSTHROUGH);  // periodic attempts are silent
    sent_count = 0;
    for (int i = 0; i < 6; i++) tick1();
    CHECK(sent_count == 6);
    check_start_burst_msgs(0);
    expect_state("wait-starting");
    CHECK(precondition_display().starting);

    // periodic attempts are one-shot: a 10s timeout goes back to MANAGED
    sent_count = 0;
    advance_until_state("managed", 11000000LL);
    CHECK(sent_count == 0);                     // no retry bursts, no stop burst
    CHECK(managed.nudge_base_ts == fake_now);
    CHECK(!precondition_display().starting);
    CHECK(!precondition_display().active);             // never confirmed started

    // A late start after the periodic timeout re-enters REQUESTED with fresh,
    // silent BMU-restart context. If it stalls, no burst is sent and a fresh
    // MANAGED interval begins.
    requested.retries = 3;                     // stale prior-attempt context
    car_status(0x05, CAN_BUS_0);
    expect_state("wait-started");
    CHECK(requested.kind == ATTEMPT_BMU_RESTART);
    CHECK(requested.retries == 0);
    CHECK(requested.last_attempt_ts == fake_now);
    CHECK(fwd(0x4E8, CAN_BUS_0, NULL) == FWD_PASSTHROUGH);
    sent_count = 0;
    advance_until_state("managed", 71000000LL);
    CHECK(sent_count == 0);
    CHECK(managed.nudge_base_ts == fake_now);

    // An observed restart that falls back to idle keeps the same 70-second
    // confirmation window, then returns to MANAGED without sending a burst.
    car_status(0x05, CAN_BUS_0);
    expect_state("wait-started");
    car_status(0x01, CAN_BUS_0);
    expect_state("wait-started");
    sent_count = 0;
    advance_until_state("managed", 71000000LL);
    CHECK(sent_count == 0);
    CHECK(managed.nudge_base_ts == fake_now);

    // A fully-started report in MANAGED takes the same clean re-entry path,
    // then routes directly to ACTIVE without sending a start burst.
    requested.retries = 3;
    sent_count = 0;
    car_status(0x15, CAN_BUS_0);
    expect_state("active");
    CHECK(requested.kind == ATTEMPT_BMU_RESTART);
    CHECK(requested.retries == 0);
    CHECK(sent_count == 0);
    car_status(0x01, CAN_BUS_0);
    expect_state("managed");

    // next interval: the latest status at the end of the burst determines
    // routing rather than the highest status observed during it
    advance_until_state("start-burst", 302000000LL);
    car_status(0x15, CAN_BUS_0);
    car_status(0x01, CAN_BUS_0);
    for (int i = 0; i < 6; i++) tick1();        // burst still runs to completion
    expect_state("wait-starting");
    car_status(0x15, CAN_BUS_0);
    expect_state("active");

    // another completed session returns to the repeating-mode resting state
    car_status(0x01, CAN_BUS_0);
    expect_state("managed");
    CHECK(!precondition_display().active);       // managed is not ACTIVE

    // toggle while managed: latch off, active stop
    fake_now += 2000000;                        // clear the start/stop debounce
    sent_count = 0;
    toggle();
    expect_state("stop-burst");
    CHECK(!precondition_display().active);
    CHECK(strcmp(popup_text, "Continuous: stopping") == 0);
    for (int i = 0; i < 6; i++) tick1();
    check_stop_burst_msgs(0);
    expect_state("wait-stopped");
    car_status(0x01, CAN_BUS_0);
    expect_state("idle");
    CHECK(!precondition_display().active);
    // latch is off: idle status + 5 minutes produce nothing
    sent_count = 0;
    advance_us(310000000LL);
    CHECK(sent_count == 0);
    expect_state("idle");
    CHECK(fwd(0x0C7, CAN_BUS_0, NULL) == FWD_PASSTHROUGH);

    // car-off while active drops the latch entirely (continuous)
    toggle();
    for (int i = 0; i < 6; i++) tick1();
    car_status(0x05, CAN_BUS_0);
    car_status(0x15, CAN_BUS_0);
    expect_state("active");
    CHECK(precondition_display().active);
    CHECK(!precondition_display().starting);
    int popup_count_before_car_off = popup_show_count;
    car_power(false);
    expect_state("idle");
    CHECK(!precondition_display().active);
    CHECK(idle.continuous_disabled_by_car_off);
    CHECK(popup_show_count == popup_count_before_car_off);
    sent_count = 0;
    car_power(true);                            // arm notice, but do not restart
    expect_state("idle");
    CHECK(idle.continuous_disabled_by_car_off);
    CHECK(popup_show_count == popup_count_before_car_off);

    advance_us(PRECONDITION_CAR_START_DELAY_US - 40000);
    expect_state("idle");
    CHECK(idle.continuous_disabled_by_car_off);
    CHECK(popup_show_count == popup_count_before_car_off);

    tick1();
    expect_state("idle");
    CHECK(!idle.continuous_disabled_by_car_off);
    CHECK(popup_show_count == popup_count_before_car_off + 1);
    CHECK(strcmp(popup_text,
                 "Continuous: disabled by car restart") == 0);
    car_status(0x01, CAN_BUS_0);
    advance_us(310000000LL);
    CHECK(sent_count == 0);

    // give-up with the latch on parks in MANAGED (no silent stop burst)
    toggle();
    expect_state("start-burst");
    for (int i = 0; i < 6; i++) tick1();
    expect_state("wait-starting");
    for (int r = 1; r <= 4; r++) {
        advance_until_state("start-burst", 11000000LL);
        CHECK(requested.retries == r);
        for (int i = 0; i < 6; i++) tick1();
        expect_state("wait-starting");
        CHECK(precondition_display().starting);
    }
    sent_count = 0;
    advance_until_state("managed", 11000000LL);
    CHECK(sent_count == 0);                     // no stop burst on repeating give-up
    CHECK(!precondition_display().active);             // start never confirmed: no "active"
    CHECK(fwd(0x0C7, CAN_BUS_0, NULL) == FWD_BLOCK);  // MITM stays latched
}

static void run_persistent(void) {
    precondition_init();
    expect_state("idle");
    battery_temperature(7, 10);
    car_power(true);
    expect_state("idle");                       // nothing stored in flash yet

    // manual start mirrors the latch to flash
    toggle();
    expect_state("start-burst");
    CHECK(strcmp(popup_text,
                 "Persistent: maintaining 21°C (7°C)") == 0);
    CHECK(!fake_nvs_exists);
    fake_now += 40000;
    precondition_tick();
    CHECK(!fake_nvs_exists);                    // state-machine task never writes flash
    (void)flush_pending_settings();              // lower-priority worker mirrors the latch
    CHECK(fake_nvs_exists && fake_nvs_value == 1);
    for (int i = 0; i < 5; i++) tick1();
    car_status(0x05, CAN_BUS_0);
    car_status(0x15, CAN_BUS_0);
    expect_state("active");
    CHECK(precondition_display().active);
    CHECK(!precondition_display().starting);

    // car off: the enabled session rests in MANAGED, latch stays in flash
    unsigned int commits_before = fake_nvs_commit_count;
    car_power(false);
    expect_state("managed");
    CHECK(!precondition_display().active);             // parked: no active session shown
    CHECK(fake_nvs_value == 1);
    CHECK(fwd(0x0C7, CAN_BUS_0, NULL) == FWD_BLOCK);   // MITM still latched
    // no nudges while the car is off
    sent_count = 0;
    car_status(0x01, CAN_BUS_0);                // arms idle, but car not ready
    advance_us(310000000LL);
    CHECK(sent_count == 0);
    expect_state("managed");

    // car back to READY: leave time for the car to finish coming up before the
    // relaunch, then show the countdown as a reminder
    sent_count = 0;
    int popup_count_before_restart = popup_show_count;
    car_power(true);
    expect_state("car-start-delay");
    CHECK(fwd(0x4E8, CAN_BUS_0, NULL) == FWD_PASSTHROUGH);
    advance_us(PRECONDITION_CAR_START_DELAY_US - 40000);
    expect_state("car-start-delay");
    CHECK(sent_count == 0);
    tick1();
    expect_state("start-burst");
    CHECK(requested.kind == ATTEMPT_CAR_START);
    CHECK(popup_show_count == popup_count_before_restart + 1);
    CHECK(strcmp(popup_text,
                 "Persistent: maintaining 21°C (7°C)") == 0);
    CHECK(fwd(0x4E8, CAN_BUS_0, NULL) == FWD_MODIFIED);
    for (int i = 0; i < 6; i++) tick1();
    CHECK(sent_count == 6);
    check_start_burst_msgs(0);
    car_status(0x05, CAN_BUS_0);
    car_status(0x15, CAN_BUS_0);
    expect_state("active");
    CHECK(precondition_display().active);
    CHECK(!precondition_display().starting);

    // BMU stops: the 5-minute re-nudge re-enters REQUESTED; neither it nor the
    // relaunch above changes the mirrored latch, so neither writes flash
    car_status(0x01, CAN_BUS_0);
    expect_state("managed");
    advance_until_state("start-burst", 302000000LL);
    CHECK(requested.kind == ATTEMPT_PERIODIC);
    for (int i = 0; i < 6; i++) tick1();
    car_status(0x05, CAN_BUS_0);
    car_status(0x15, CAN_BUS_0);
    expect_state("active");
    car_status(0x01, CAN_BUS_0);
    expect_state("managed");
    CHECK(fake_nvs_commit_count == commits_before);

    // toggle off from managed clears the flash latch and stops actively
    fake_now += 2000000;
    toggle();
    expect_state("stop-burst");
    CHECK(!precondition_display().active);
    CHECK(!repeating_mode_enabled());
    CHECK(strcmp(popup_text, "Persistent: stopping") == 0);
    tick1();
    CHECK(fake_nvs_value == 0);                 // lower-priority worker flushed the clear
    for (int i = 0; i < 5; i++) tick1();
    expect_state("wait-stopped");
    car_status(0x01, CAN_BUS_0);
    expect_state("idle");
    // next car power cycle: nothing relaunches
    car_power(false);
    car_power(true);
    expect_state("idle");
}

// fresh process with the flash latch already set: a WiCAN power cycle while
// persistent preconditioning was enabled
static void run_persistent_restore(void) {
    fake_nvs_exists = true;
    fake_nvs_value = 1;
    precondition_init();
    expect_state("managed");                    // restored latch parks in MANAGED
    CHECK(requested.kind == ATTEMPT_RESTORE);
    CHECK(popup_show_count == 0);                // parked restore is not an attempt yet
    CHECK(!precondition_display().active);             // car not ready at boot: no session
    CHECK(fwd(0x0C7, CAN_BUS_0, NULL) == FWD_BLOCK);  // MITM guards the latch from boot
    // A late status report moves a restored session into ACTIVE, then its
    // idle edge returns to MANAGED without clearing the persistent latch.
    car_status(0x15, CAN_BUS_0);
    expect_state("active");
    CHECK(requested.kind == ATTEMPT_BMU_RESTART);
    CHECK(requested.retries == 0);
    car_status(0x01, CAN_BUS_0);
    expect_state("managed");
    // no nudges before the car is ready
    sent_count = 0;
    advance_us(310000000LL);
    CHECK(sent_count == 0);
    expect_state("managed");
    int popup_count_before_restart = popup_show_count;
    car_power(true);
    expect_state("car-start-delay");            // first ready edge starts the delay
    CHECK(fwd(0x4E8, CAN_BUS_0, NULL) == FWD_PASSTHROUGH);  // no countdown before the try
    advance_us(PRECONDITION_CAR_START_DELAY_US - 40000);
    expect_state("car-start-delay");
    CHECK(sent_count == 0);
    tick1();
    expect_state("start-burst");
    CHECK(requested.kind == ATTEMPT_CAR_START);
    CHECK(popup_show_count == popup_count_before_restart + 1);
    CHECK(strcmp(popup_text, "Persistent: maintaining 21°C") == 0);
    CHECK(fwd(0x4E8, CAN_BUS_0, NULL) == FWD_MODIFIED);  // countdown shown
    for (int i = 0; i < 6; i++) tick1();
    car_status(0x05, CAN_BUS_0);
    car_status(0x15, CAN_BUS_0);
    expect_state("active");
    CHECK(precondition_display().active);
    CHECK(!precondition_display().starting);
    CHECK(fake_nvs_commit_count == 0);          // the restore itself never rewrites flash
}

static void run_persistent_disable_before_ready(void) {
    fake_nvs_exists = true;
    fake_nvs_value = 1;
    precondition_init();
    expect_state("managed");

    // within the debounce window after boot the toggle is swallowed
    toggle();
    expect_state("managed");
    CHECK(repeating_mode_enabled());

    fake_now += 2000000;
    toggle();
    expect_state("stop-burst");                 // disable = active stop, even before ready
    CHECK(!repeating_mode_enabled());
    tick1();
    CHECK(fake_nvs_value == 0);                 // latch clear flushed to flash
    for (int i = 0; i < 5; i++) tick1();
    expect_state("idle");                       // no status frame ever seen: no wait

    car_power(true);
    expect_state("idle");                       // disabled: ready must not relaunch
}

static void run_persistent_write_retry(void) {
    precondition_init();
    expect_state("idle");

    toggle();
    expect_state("start-burst");
    tick1();
    CHECK(fake_nvs_value == 1);

    fake_now += 2000000;
    fake_nvs_commit_failures = 1;
    toggle();
    expect_state("stop-burst");
    CHECK(!repeating_mode_enabled());
    tick1();                                    // first flush attempt fails
    CHECK(fake_nvs_commit_failures == 0);
    CHECK(fake_nvs_value == 1);                 // failed commit left flash stale

    advance_us(PERSISTENT_SETTINGS_RETRY_US - 80000);
    CHECK(fake_nvs_value == 1);                 // still inside the backoff
    advance_us(160000);
    CHECK(fake_nvs_value == 0);                 // retry landed

    car_power(true);
    expect_state("idle");                      // the stale enable cannot relaunch
}

static void run_persistent_write_retry_limit(void) {
    precondition_init();
    toggle();
    tick1();
    CHECK(fake_nvs_value == 1);

    fake_now += 2000000;
    fake_nvs_commit_failures = PERSISTENT_SETTINGS_MAX_RETRIES + 2U;
    toggle();

    // initial attempt + MAX_RETRIES retries, then the budget is spent
    advance_us((PERSISTENT_SETTINGS_MAX_RETRIES + 1U) * PERSISTENT_SETTINGS_RETRY_US);
    CHECK(mirrored_u8_settings[MIRRORED_U8_PRECONDITIONING_ENABLED].save_failures
          == PERSISTENT_SETTINGS_MAX_RETRIES + 1U);
    CHECK(fake_nvs_value == 1);                 // flash left stale
    unsigned int failures_left = fake_nvs_commit_failures;
    CHECK(failures_left == 1U);

    advance_us(2 * PERSISTENT_SETTINGS_RETRY_US);
    CHECK(fake_nvs_commit_failures == failures_left);  // no writes after the cap
    CHECK(fake_nvs_value == 1);
}

// a stored persistent latch must not leak into other modes
static void run_once_ignores_stored_latch(void) {
    fake_nvs_exists = true;
    fake_nvs_value = 1;
    precondition_init();
    car_power(true);
    expect_state("idle");                       // no relaunch in once mode
    toggle();
    for (int i = 0; i < 6; i++) tick1();
    car_status(0x05, CAN_BUS_0);
    car_status(0x15, CAN_BUS_0);
    expect_state("active");                     // not managed
    CHECK(precondition_display().active);
}

// ---- suite table ----
// each suite runs in its own forked process: the config snapshot, repeating
// latch, fake NVS, and platform discovery flags all live in process statics
typedef struct {
    const char *name;
    int8_t mode;
    int8_t press;
    void (*fn)(void);
} suite_t;

static const suite_t suites[] = {
    {"precondition short-press once", ONCE, PRESS_SHORT, run_short_press},
    {"precondition long-press once", ONCE, PRESS_LONG, run_long_press},
    {"precondition battery temperature cutoff", ONCE, PRESS_SHORT, run_battery_temperature_cutoff},
    {"precondition battery state of charge", ONCE, PRESS_SHORT, run_battery_soc},
    {"precondition automatic temperature cutoff", CONTINUOUS, PRESS_SHORT, run_automatic_temperature_cutoff},
    {"precondition persistent temperature cutoff", PERSISTENT, PRESS_SHORT, run_automatic_temperature_cutoff},
    {"precondition automatic SoC cutoff", CONTINUOUS, PRESS_SHORT, run_automatic_soc_cutoff},
    {"precondition continuous SoC cutoff message", CONTINUOUS, PRESS_SHORT, run_repeating_soc_cutoff_message},
    {"precondition persistent SoC cutoff message", PERSISTENT, PRESS_SHORT, run_repeating_soc_cutoff_message},
    {"precondition continuous temperature notice", CONTINUOUS, PRESS_SHORT, run_repeating_temperature_notice},
    {"precondition persistent temperature notice", PERSISTENT, PRESS_SHORT, run_repeating_temperature_notice},
    {"precondition once stopping notices", ONCE, PRESS_SHORT, run_once_stopping_notices},
    {"precondition concurrent dispatch", ONCE, PRESS_LONG, run_concurrent_dispatch},
    {"precondition continuous", CONTINUOUS, PRESS_SHORT, run_continuous},
    {"precondition persistent", PERSISTENT, PRESS_SHORT, run_persistent},
    {"precondition persistent restore", PERSISTENT, PRESS_SHORT, run_persistent_restore},
    {"precondition persistent disable before ready", PERSISTENT, PRESS_SHORT, run_persistent_disable_before_ready},
    {"precondition persistent write retry", PERSISTENT, PRESS_SHORT, run_persistent_write_retry},
    {"precondition persistent write retry limit", PERSISTENT, PRESS_SHORT, run_persistent_write_retry_limit},
    {"precondition once ignores stored latch", ONCE, PRESS_SHORT, run_once_ignores_stored_latch},
};
#define NUM_SUITES (sizeof(suites) / sizeof(suites[0]))

static int run_suite(const suite_t *s) {
    cfg_mode = s->mode;
    cfg_press = s->press;
    s->fn();
    return test_report(s->name);
}

int main(int argc, char **argv) {
    // single-suite run, handy for debugging: test_precondition <substring>
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
