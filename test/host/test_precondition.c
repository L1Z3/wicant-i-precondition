// host-side behavioral test: drives the precondition state machine with a
// fake clock and a recording can_send
//
// precondition.c is #included (not linked) so the test can inspect states
// and per-state context directly.
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <sys/wait.h>
#include <unistd.h>
#include "test_support.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "can.h"
#include "config_server.h"

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

static int8_t cfg_button = SW_STAR;
static int8_t cfg_mode = ONCE;
static int8_t cfg_press = PRESS_SHORT;
int8_t config_server_precon_button(void) { return cfg_button; }
int8_t config_server_precon_mode(void) { return cfg_mode; }
int8_t config_server_precon_press(void) { return cfg_press; }

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

static void tick1(void) { fake_now += 40000; precondition_tick(); }
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

static void run_short_press(void) {
    precondition_init();
    expect_state("idle");

    // --- platform without status frames: no retries, fake-active at 70s ---
    sent_count = 0;
    toggle();
    expect_state("start-burst");
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
    CHECK(fwd(0x4E8, CAN_BUS_0, NULL) == FWD_PASSTHROUGH);  // display off in active
    CHECK(fwd(0x0C7, CAN_BUS_0, NULL) == FWD_BLOCK);        // still blocking
    // stop without status frames: burst then straight to idle, no display
    fake_now += 2000000;
    toggle();
    expect_state("stop-burst");
    CHECK(fwd(0x4E8, CAN_BUS_0, NULL) == FWD_PASSTHROUGH);
    sent_count = 0;
    for (int i = 0; i < 6; i++) tick1();
    CHECK(sent_count == 6);
    check_stop_burst_msgs(0);
    expect_state("idle");

    // --- happy path with status frames ---
    toggle();
    expect_state("start-burst");
    car_status(0x01, CAN_BUS_1);               // head unit bus: must be ignored
    CHECK(!platform.status_frame_available);
    car_status(0x01, CAN_BUS_0);               // idle status during burst: ignored but latched
    CHECK(platform.status_frame_available);
    expect_state("start-burst");
    for (int i = 0; i < 6; i++) tick1();
    expect_state("wait-starting");
    CHECK(fwd(0x4E8, CAN_BUS_0, &m) == FWD_MODIFIED);
    CHECK((m.data[6] & 0x0F) == 0x1);          // FLAG_BLUE_1 before starting confirm
    car_status(0x45, CAN_BUS_0);               // EV6-style "starting"
    expect_state("wait-started");
    CHECK(fwd(0x4E8, CAN_BUS_0, &m) == FWD_MODIFIED);
    CHECK((m.data[6] & 0x0F) == 0x0);          // FLAG_DESTINATION after starting confirm
    car_status(0x55, CAN_BUS_0);               // EV6-style "started"
    expect_state("active");
    // downgrade: active -> starting again
    car_status(0x05, CAN_BUS_0);
    expect_state("wait-started");
    car_status(0x15, CAN_BUS_0);
    expect_state("active");
    // "complete" in ONCE mode -> real stop
    cfg_mode = ONCE;
    car_status(0x01, CAN_BUS_0);
    expect_state("stop-burst");
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

    // --- continuous mode "complete" -> passive reset ---
    cfg_mode = CONTINUOUS;
    toggle();
    for (int i = 0; i < 6; i++) tick1();
    car_status(0x05, CAN_BUS_0);
    car_status(0x15, CAN_BUS_0);
    expect_state("active");
    sent_count = 0;
    car_status(0x01, CAN_BUS_0);
    expect_state("idle");
    advance_us(1000000);
    CHECK(sent_count == 0);                    // no stop messages sent
    cfg_mode = ONCE;

    // --- debounce ---
    toggle();
    expect_state("start-burst");
    toggle();                                   // within 1s: debounced away
    CHECK(sm_in(&precon_sm, &S_REQUESTED));
    advance_us(1200000);
    expect_state("wait-starting");
    toggle();                                   // past debounce: stop
    expect_state("stop-burst");
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
    for (int r = 1; r <= 4; r++) {
        advance_until_state("start-burst", 11000000);   // >10s: retry
        CHECK(requested.retries == r);
        for (int i = 0; i < 6; i++) tick1();
        expect_state("wait-starting");
    }
    CHECK(sent_count == 5 * 6);
    CHECK(fwd(0x4E8, CAN_BUS_0, &m) == FWD_MODIFIED);
    CHECK((m.data[0] >> 4) == 4);               // retry count in tenths digit
    // retries exhausted: give up with a silent stop
    int base = sent_count;
    advance_until_state("stop-burst", 11000000);
    CHECK(stopping.retries == 4);
    CHECK(fwd(0x4E8, CAN_BUS_0, NULL) == FWD_PASSTHROUGH);  // silent stop: no display
    for (int i = 0; i < 6; i++) tick1();
    expect_state("wait-stopped");
    CHECK(sent_count == base + 6);
    check_stop_burst_msgs(base);
    advance_us(10100000);                       // no stop retries in the give-up case
    expect_state("idle");
    CHECK(sent_count == base + 6);

    // --- wait-started timeout retries at 70s, confirm mid-burst ---
    toggle();
    for (int i = 0; i < 6; i++) tick1();
    car_status(0x05, CAN_BUS_0);
    expect_state("wait-started");
    advance_us(10100000);
    expect_state("wait-started");               // 10s retry does NOT apply here
    advance_until_state("start-burst", 61000000);  // ...but 70s does
    CHECK(requested.retries == 1);
    sent_count = 0;
    for (int i = 0; i < 3; i++) tick1();        // mid-burst
    expect_state("start-burst");
    car_status(0x15, CAN_BUS_0);                // started confirm mid-burst
    expect_state("start-burst");                // burst still runs to completion
    CHECK(fwd(0x4E8, CAN_BUS_0, NULL) == FWD_PASSTHROUGH);  // but display is off
    for (int i = 0; i < 3; i++) tick1();
    CHECK(sent_count == 6);                     // full burst sent despite confirm
    expect_state("active");
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

static int run_mode(bool longpress) {
    cfg_press = longpress ? PRESS_LONG : PRESS_SHORT;
    if (longpress) run_long_press();
    else run_short_press();
    return test_report(longpress ? "precondition long-press" : "precondition short-press");
}

int main(int argc, char **argv) {
    // single-mode run, handy for debugging: test_precondition short|long
    if (argc > 1) {
        return run_mode(strcmp(argv[1], "long") == 0);
    }
    // cached_precon_press_type() latches the config on its first use, so each
    // press mode needs a fresh process: short-press runs in a child, then
    // long-press runs here
    // TODO(ejones): redesign the caching in a way where this isn't a problem
    pid_t child = fork();
    if (child < 0) {
        perror("fork");
        return 1;
    }
    if (child == 0) {
        exit(run_mode(false));
    }
    int status = 0;
    waitpid(child, &status, 0);
    bool short_failed = !WIFEXITED(status) || WEXITSTATUS(status) != 0;
    bool long_failed = run_mode(true) != 0;
    return (short_failed || long_failed) ? 1 : 0;
}
