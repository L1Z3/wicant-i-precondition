#include "beep.h"
#include "can.h"
#include "config_server.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#define TAG "beep"

// Nominal time between beep starts, including across queued requests.
#define BEEP_INTERVAL_MS 250U
// Preserve the recorded 10 ms between the 0x0C start and 0x04 release frames.
#define BEEP_RELEASE_DELAY_MS 10U
#define BEEP_FRAME_ID 0x465U
#define BEEP_QUEUE_DEPTH 4U
#define BEEP_SEND_ATTEMPTS 3U
#define BEEP_SEND_RETRY_MS 10U
#define BEEP_TASK_STACK_SIZE (2U * 1024U)
#define BEEP_TASK_PRIORITY 5U

_Static_assert(BEEP_INTERVAL_MS > BEEP_RELEASE_DELAY_MS,
               "beep interval must leave time after the release frame");
_Static_assert(pdMS_TO_TICKS(BEEP_RELEASE_DELAY_MS) > 0U
               && pdMS_TO_TICKS(BEEP_INTERVAL_MS - BEEP_RELEASE_DELAY_MS) > 0U,
               "beep timing must fit the FreeRTOS tick resolution");

static QueueHandle_t beep_queue;
static can_bus_t beep_bus;

static bool send_frame(uint8_t value) {
    twai_message_t frame = {
        .identifier = BEEP_FRAME_ID,
        .data_length_code = 8U,
        .data = {0, 0, 0, 0, 0, 0, value, 0},
    };
    for (unsigned attempt = 0U; attempt < BEEP_SEND_ATTEMPTS; attempt++) {
        if (can_send(beep_bus, &frame, 0) == 0) {
            return true;
        }
        if (attempt + 1U < BEEP_SEND_ATTEMPTS) {
            vTaskDelay(pdMS_TO_TICKS(BEEP_SEND_RETRY_MS));
        }
    }
    ESP_LOGW(TAG, "could not send beep frame 0x%02X", value);
    return false;
}

static void play_sequence(uint8_t count) {
    for (unsigned i = 0U; i < count; i++) {
        bool started = send_frame(0x0CU);
        vTaskDelay(pdMS_TO_TICKS(BEEP_RELEASE_DELAY_MS));
        // Attempt the release even if the start failed; abort further beeps
        // after a persistent send failure so a disabled bus cannot stall us.
        bool released = send_frame(0x04U);
        vTaskDelay(pdMS_TO_TICKS(BEEP_INTERVAL_MS - BEEP_RELEASE_DELAY_MS));
        if (!started || !released) {
            return;
        }
    }
}

static void beep_task(void *arg) {
    (void)arg;
    for (;;) {
        uint8_t count;
        if (xQueueReceive(beep_queue, &count, portMAX_DELAY) == pdTRUE) {
            play_sequence(count);
        }
    }
}

void beep_init(void) {
    // TODO(ejones): use HEAD_UNIT_BUS here (w/ change that updates that based on v300 vs custom)
    // The head unit is on bus 1 in bridge mode, bus 0 in parallel mode or on
    // single-bus hardware. Wiring configuration changes reboot the unit.
    beep_bus = CAN_BUS_0;
#if CAN_BUS_COUNT > 1
    if (config_server_get_fwd_en() != 0) {
        beep_bus = CAN_BUS_1;
    }
#endif
    beep_queue = xQueueCreate(BEEP_QUEUE_DEPTH, sizeof(uint8_t));
    configASSERT(beep_queue != NULL);
    BaseType_t created = xTaskCreate(beep_task, "beep", BEEP_TASK_STACK_SIZE,
                                     NULL, BEEP_TASK_PRIORITY, NULL);
    configASSERT(created == pdPASS);
}

bool beep_play(uint8_t count) {
    return beep_queue != NULL && count > 0U
        && xQueueSend(beep_queue, &count, 0) == pdTRUE;
}
