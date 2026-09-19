// Drive the independent beep worker's playback with a fake clock and CAN bus.
#include <stdlib.h>
#include <string.h>
#include "test_support.h"
#include "beep.h"
#include "can.h"
#include "freertos/queue.h"

// Build with -DCAN_BUS_COUNT=1 as well to cover single-bus routing.
#ifndef CAN_BUS_COUNT
#define CAN_BUS_COUNT 2
#endif

typedef struct {
    can_bus_t bus;
    twai_message_t frame;
    int64_t at_us;
} sent_t;

static sent_t sent[512];
static size_t sent_count;
static uint8_t failing_value;
static unsigned failures_remaining;
static int8_t bridge_enabled;

int8_t config_server_get_fwd_en(void) { return bridge_enabled; }

esp_err_t can_send(can_bus_t bus, twai_message_t *frame, TickType_t wait) {
    CHECK(wait == 0U);
    CHECK(sent_count < sizeof(sent) / sizeof(sent[0]));
    if (sent_count < sizeof(sent) / sizeof(sent[0])) {
        sent[sent_count++] = (sent_t){bus, *frame, fake_now};
    }
    if (frame->data[6] == failing_value && failures_remaining > 0U) {
        failures_remaining--;
        return -1;
    }
    return 0;
}

void vTaskDelay(TickType_t ticks) { fake_now += ticks * 1000LL; }

typedef struct {
    uint8_t *data;
    unsigned capacity;
    unsigned head;
    unsigned count;
} fake_queue_t;

QueueHandle_t xQueueCreate(UBaseType_t length, UBaseType_t item_size) {
    CHECK(item_size == sizeof(uint8_t));
    fake_queue_t *queue = calloc(1, sizeof(*queue));
    queue->data = calloc(length, item_size);
    queue->capacity = length;
    return queue;
}

BaseType_t xQueueSend(QueueHandle_t handle, const void *item, TickType_t wait) {
    fake_queue_t *queue = handle;
    CHECK(wait == 0U);
    if (queue->count == queue->capacity) {
        return pdFALSE;
    }
    unsigned tail = (queue->head + queue->count) % queue->capacity;
    queue->data[tail] = *(const uint8_t *)item;
    queue->count++;
    return pdTRUE;
}

BaseType_t xQueueReceive(QueueHandle_t handle, void *item, TickType_t wait) {
    (void)wait;
    fake_queue_t *queue = handle;
    if (queue->count == 0U) {
        return pdFALSE;
    }
    *(uint8_t *)item = queue->data[queue->head];
    queue->head = (queue->head + 1U) % queue->capacity;
    queue->count--;
    return pdTRUE;
}

#include "beep.c"

static void reset(bool bridge) {
    if (beep_queue != NULL) {
        fake_queue_t *queue = beep_queue;
        free(queue->data);
        free(queue);
    }
    fake_now = 0;
    sent_count = 0U;
    failures_remaining = 0U;
    bridge_enabled = bridge;
    beep_init();
}

// The host's xTaskCreate does not run an infinite worker; consume one request
// and execute its playback synchronously, with delays advancing the fake clock.
static void play_next(void) {
    uint8_t count = 0U;
    CHECK(xQueueReceive(beep_queue, &count, 0) == pdTRUE);
    play_sequence(count);
}

static void expect_beeps(size_t count, can_bus_t bus) {
    CHECK(sent_count == count * 2U);
    for (size_t i = 0U; i < sent_count; i++) {
        uint8_t expected[8] = {0, 0, 0, 0, 0, 0, i % 2U ? 0x0CU : 0x04U, 0};
        CHECK(sent[i].bus == bus);
        CHECK(sent[i].frame.identifier == 0x465U);
        CHECK(sent[i].frame.data_length_code == 8U);
        CHECK(memcmp(sent[i].frame.data, expected, sizeof(expected)) == 0);
        int64_t expected_ms = (i / 2U) * BEEP_INTERVAL_MS
                             + (i % 2U) * BEEP_RELEASE_DELAY_MS;
        CHECK(sent[i].at_us == expected_ms * 1000LL);
    }
}

static void test_counts_and_routing(void) {
    const uint8_t counts[] = {1U, 2U, 3U, 7U, UINT8_MAX};
    for (unsigned bridge = 0U; bridge <= 1U; bridge++) {
        for (size_t i = 0U; i < sizeof(counts); i++) {
            reset(bridge);
            CHECK(!beep_play(0U));
            CHECK(beep_play(counts[i]));
            CHECK(sent_count == 0U && fake_now == 0);  // Caller never waits for playback.
            play_next();
            can_bus_t expected_bus = bridge && CAN_BUS_COUNT > 1
                                    ? CAN_BUS_1 : CAN_BUS_0;
            expect_beeps(counts[i], expected_bus);
        }
    }
}

static void test_queue(void) {
    reset(false);
    for (unsigned i = 1U; i <= BEEP_QUEUE_DEPTH; i++) {
        CHECK(beep_play(i));
    }
    CHECK(!beep_play(1U));
    CHECK(sent_count == 0U);
    size_t total = 0U;
    for (unsigned i = 1U; i <= BEEP_QUEUE_DEPTH; i++) {
        play_next();
        total += i;
        expect_beeps(total, CAN_BUS_0);  // Includes spacing across requests.
    }
    CHECK(beep_play(1U));  // Queue space is reusable after playback.
    play_next();
    expect_beeps(total + 1U, CAN_BUS_0);
}

static void test_send_failures(void) {
    // A transient release failure retries the release, never another start.
    reset(false);
    failing_value = 0x0CU;
    failures_remaining = 1U;
    CHECK(beep_play(2U));
    play_next();
    CHECK(sent_count == 5U);
    CHECK(sent[0].frame.data[6] == 0x04U);
    CHECK(sent[1].frame.data[6] == 0x0CU && sent[2].frame.data[6] == 0x0CU);
    CHECK(sent[2].at_us - sent[1].at_us == BEEP_SEND_RETRY_MS * 1000LL);
    CHECK(sent[3].frame.data[6] == 0x04U && sent[4].frame.data[6] == 0x0CU);
    CHECK(sent[3].at_us - sent[0].at_us >= BEEP_INTERVAL_MS * 1000LL);

    // Persistent failures abort the sequence after bounded attempts, still
    // attempt release, and allow the next queued request to run.
    const uint8_t values[] = {0x04U, 0x0CU};
    for (size_t i = 0U; i < sizeof(values); i++) {
        reset(false);
        failing_value = values[i];
        failures_remaining = BEEP_SEND_ATTEMPTS;
        CHECK(beep_play(3U));
        CHECK(beep_play(1U));
        play_next();
        CHECK(sent_count == BEEP_SEND_ATTEMPTS + 1U);
        CHECK(sent[sent_count - 1U].frame.data[6] == 0x0CU);
        size_t before = sent_count;
        play_next();
        CHECK(sent_count == before + 2U);
        CHECK(sent[before].frame.data[6] == 0x04U);
        CHECK(sent[before + 1U].frame.data[6] == 0x0CU);
    }
}

int main(void) {
    CHECK(!beep_play(1U));
    test_counts_and_routing();
    test_queue();
    test_send_failures();
    return test_report("head-unit beep service");
}
