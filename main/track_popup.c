#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "esp_log.h"
#include "isotp_tx.h"
#include "track_popup.h"
#include "utf8_utf16_converter.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#define TAG "track_popup"

// ********************* popup and transport configuration *********************

// currently just use "Sounds of Nature" category
#define TRACK_POPUP_MEDIA_FRAME_ID 0x4CEU
#define TRACK_POPUP_MEDIA_TYPE 0x04U
#define TRACK_POPUP_ISOTP_TX_ID 0x6E0U
#define TRACK_POPUP_ISOTP_FLOW_CONTROL_ID 0x6BEU
#define TRACK_POPUP_TARGET_BUS CAN_BUS_0
#define TRACK_POPUP_SOURCE_BUS CAN_BUS_1

#define TRACK_POPUP_TRIGGER_FRAME_COUNT 3U
#define TRACK_POPUP_TRIGGER_SETTLE_US 5000U
#define TRACK_POPUP_TRIGGER_TIMEOUT_US 2000000U
#define TRACK_POPUP_DISPLAY_HOLD_US 5000000U
#define TRACK_POPUP_MAX_TEXT_CHARACTERS 64U
#define TRACK_POPUP_MAX_TEXT_BYTES \
    (TRACK_POPUP_MAX_TEXT_CHARACTERS * sizeof(utf16_t))
#define TRACK_POPUP_QUEUE_DEPTH 2U

#define TRACK_POPUP_ISOTP_FLOW_CONTROL_TIMEOUT_US 1000000U
#define TRACK_POPUP_ISOTP_MAX_WAIT_FRAMES 3U
#define TRACK_POPUP_ISOTP_TASK_STACK_SIZE (3U * 1024U)
#define TRACK_POPUP_ISOTP_TASK_PRIORITY 6U

// ********************* state machine storage *********************

typedef struct {
    size_t size;
    uint8_t data[TRACK_POPUP_MAX_TEXT_BYTES];
} track_popup_request_t;

typedef struct {
    sm_t sm;
    QueueHandle_t queue;
    isotp_tx_t isotp;
    track_popup_request_t pending_request;
    bool media_type_owned;
} track_popup_t;

typedef struct {
    track_popup_request_t request;
    uint8_t trigger_frames_remaining;
    int64_t requested_at_us;
    int64_t trigger_forwarded_at_us;
} trigger_ctx_t;

static track_popup_t popup;
static trigger_ctx_t trigger_ctx;  // owned by TRIGGER state
static const sm_state_t S_IDLE, S_TRIGGER, S_SENDING, S_HOLD;

_Static_assert(offsetof(track_popup_t, sm) == 0,
               "sm must be the first track_popup_t field");

static track_popup_t *owner(sm_t *sm) {
    return (track_popup_t *)sm;
}

// ********************* UTF-16LE text encoding *********************

// Convert a NUL-terminated UTF-8 string to unadorned UTF-16LE: 
// no byte order mark (BOM) and no terminating UTF-16 NUL.
// Malformed input becomes U+FFFD. Reject overflow as a whole; never place
// truncated text on the queue.
static bool encode_text(const char *text, track_popup_request_t *out) {
    if (text == NULL || out == NULL || text[0] == '\0') {
        return false;
    }

    const utf8_t *utf8 = (const utf8_t *)text;
    size_t utf8_size = strlen(text);
    utf16_t utf16[TRACK_POPUP_MAX_TEXT_BYTES / sizeof(utf16_t)];
    size_t utf16_capacity = sizeof(utf16) / sizeof(utf16[0]);
    size_t utf16_size = utf8_to_utf16(utf8, utf8_size, NULL, 0U);
    if (utf16_size == 0U || utf16_size > utf16_capacity
            || utf8_to_utf16(utf8, utf8_size, utf16, utf16_capacity)
                    != utf16_size) {
        return false;
    }

    for (size_t i = 0U; i < utf16_size; i++) {
        out->data[i * 2U] = (uint8_t)(utf16[i] & 0xFFU);
        out->data[i * 2U + 1U] = (uint8_t)(utf16[i] >> 8U);
    }
    out->size = utf16_size * sizeof(utf16_t);
    return true;
}

// ********************* shared state helpers *********************

static const char *result_name(isotp_tx_result_t result) {
    switch (result) {
        case ISOTP_TX_RESULT_SUCCESS: return "success";
        case ISOTP_TX_RESULT_TIMEOUT: return "timeout";
        case ISOTP_TX_RESULT_OVERFLOW: return "receiver overflow";
        case ISOTP_TX_RESULT_PROTOCOL_ERROR: return "protocol error";
        case ISOTP_TX_RESULT_SEND_ERROR: return "CAN send error";
        default: return "unknown";
    }
}

static fwd_result_t pin_media_type(track_popup_t *service,
                                   twai_message_t *msg, can_bus_t fwd_bus) {
    if (!service->media_type_owned
            || fwd_bus != TRACK_POPUP_TARGET_BUS
            || msg->identifier != TRACK_POPUP_MEDIA_FRAME_ID
            || msg->data_length_code < 2U) {
        return FWD_PASSTHROUGH;
    }
    msg->data[0] = TRACK_POPUP_MEDIA_TYPE;
    return FWD_MODIFIED;
}

// ********************* idle state *********************

static void idle_enter(sm_t *sm) {
    // Media-type ownership is scoped to one popup attempt. Release it after
    // the display hold finishes or any setup/transfer failure returns here.
    owner(sm)->media_type_owned = false;
}

static void idle_tick(sm_t *sm) {
    track_popup_t *service = owner(sm);
    if (xQueueReceive(service->queue, &service->pending_request, 0) == pdTRUE) {
        // Defer changing the media category until there is text ready to
        // accompany the popup.
        service->media_type_owned = true;
        sm_transition(sm, &S_TRIGGER);
    }
}

// ********************* trigger state *********************

static void trigger_enter(sm_t *sm) {
    track_popup_t *service = owner(sm);
    trigger_ctx.request = service->pending_request;
    trigger_ctx.trigger_frames_remaining = TRACK_POPUP_TRIGGER_FRAME_COUNT;
    trigger_ctx.requested_at_us = sm_now(sm);
}

static void trigger_tick(sm_t *sm) {
    track_popup_t *service = owner(sm);
    if (trigger_ctx.trigger_frames_remaining == 0U
            && sm_now(sm) - trigger_ctx.trigger_forwarded_at_us
                    >= TRACK_POPUP_TRIGGER_SETTLE_US) {
        if (isotp_tx_start(&service->isotp, trigger_ctx.request.data,
                           trigger_ctx.request.size)) {
            sm_transition(sm, &S_SENDING);
        } else {
            ESP_LOGW(TAG, "ISO-TP transmitter busy");
            sm_transition(sm, &S_IDLE);
        }
    } else if (sm_now(sm) - trigger_ctx.requested_at_us
                    >= TRACK_POPUP_TRIGGER_TIMEOUT_US) {
        ESP_LOGW(TAG, "timed out waiting for 0x4CE trigger frames");
        sm_transition(sm, &S_IDLE);
    }
}

static fwd_result_t trigger_fwd(sm_t *sm, twai_message_t *msg,
                                can_bus_t fwd_bus) {
    track_popup_t *service = owner(sm);
    fwd_result_t media_result = pin_media_type(service, msg, fwd_bus);
    if (media_result == FWD_MODIFIED) {
        if (trigger_ctx.trigger_frames_remaining > 0U) {
            msg->data[1] = 0x11U;
            trigger_ctx.trigger_frames_remaining--;
            if (trigger_ctx.trigger_frames_remaining == 0U) {
                trigger_ctx.trigger_forwarded_at_us = sm_now(sm);
            }
        }
        return FWD_MODIFIED;
    }
    if (fwd_bus == TRACK_POPUP_TARGET_BUS
            && msg->identifier == TRACK_POPUP_ISOTP_TX_ID) {
        return FWD_BLOCK;
    }
    return FWD_PASSTHROUGH;
}

// ********************* sending state *********************

static void sending_tick(sm_t *sm) {
    track_popup_t *service = owner(sm);
    if (isotp_tx_busy(&service->isotp)) {
        return;
    }
    isotp_tx_result_t result = isotp_tx_result(&service->isotp);
    if (result == ISOTP_TX_RESULT_SUCCESS) {
        ESP_LOGI(TAG, "ISO-TP transfer complete; holding popup media type");
        sm_transition(sm, &S_HOLD);
    } else {
        ESP_LOGW(TAG, "ISO-TP transfer failed: %s", result_name(result));
        sm_transition(sm, &S_IDLE);
    }
}

static void sending_rx(sm_t *sm, const twai_message_t *msg, can_bus_t rx_bus) {
    isotp_tx_rx(&owner(sm)->isotp, msg, rx_bus);
}

static fwd_result_t popup_owned_fwd(sm_t *sm, twai_message_t *msg,
                                    can_bus_t fwd_bus) {
    track_popup_t *service = owner(sm);
    fwd_result_t media_result = pin_media_type(service, msg, fwd_bus);
    if (media_result != FWD_PASSTHROUGH) {
        return media_result;
    }

    // Our injected frames bypass the bridge hook. Suppress competing head-unit
    // text and the receiver's FC responses while this request owns the pair.
    if (fwd_bus == TRACK_POPUP_TARGET_BUS
            && msg->identifier == TRACK_POPUP_ISOTP_TX_ID) {
        return FWD_BLOCK;
    }
    if (fwd_bus == TRACK_POPUP_SOURCE_BUS
            && msg->identifier == TRACK_POPUP_ISOTP_FLOW_CONTROL_ID) {
        return FWD_BLOCK;
    }
    return FWD_PASSTHROUGH;
}

// ********************* display hold state *********************

static void hold_tick(sm_t *sm) {
    if (sm_time_in_us(sm, &S_HOLD) >= TRACK_POPUP_DISPLAY_HOLD_US) {
        sm_transition(sm, &S_IDLE);
    }
}

// ********************* state definitions *********************

static const sm_state_t S_IDLE = {
    .name = "idle",
    .enter = idle_enter,
    .tick = idle_tick,
};

static const sm_state_t S_TRIGGER = {
    .name = "trigger",
    .ctx = &trigger_ctx,
    .ctx_size = sizeof(trigger_ctx),
    .enter = trigger_enter,
    .tick = trigger_tick,
    .fwd = trigger_fwd,
};

static const sm_state_t S_SENDING = {
    .name = "sending",
    .tick = sending_tick,
    .rx = sending_rx,
    .fwd = popup_owned_fwd,
};

static const sm_state_t S_HOLD = {
    .name = "hold",
    .tick = hold_tick,
    .fwd = popup_owned_fwd,
};

// ********************* public API *********************

void track_popup_init(void) {
    memset(&popup, 0, sizeof(popup));
    popup.queue = xQueueCreate(TRACK_POPUP_QUEUE_DEPTH,
                               sizeof(track_popup_request_t));
    configASSERT(popup.queue != NULL);

    const isotp_tx_config_t config = {
        .bus = TRACK_POPUP_TARGET_BUS,
        .tx_id = TRACK_POPUP_ISOTP_TX_ID,
        .flow_control_id = TRACK_POPUP_ISOTP_FLOW_CONTROL_ID,
        .flow_control_timeout_us = TRACK_POPUP_ISOTP_FLOW_CONTROL_TIMEOUT_US,
        .can_send_wait_ticks = 1,
        .max_wait_frames = TRACK_POPUP_ISOTP_MAX_WAIT_FRAMES,
        .padding_byte = 0x00U,
    };
    isotp_tx_init(&popup.isotp, "track-popup-isotp", &config);
    bool worker_started = isotp_tx_start_worker(
        &popup.isotp, "track_popup_isotp",
        TRACK_POPUP_ISOTP_TASK_STACK_SIZE, TRACK_POPUP_ISOTP_TASK_PRIORITY);
    configASSERT(worker_started);
    sm_init(&popup.sm, "track-popup", &S_IDLE, NULL);
}

void track_popup_tick(void) {
    if (popup.queue != NULL) {
        sm_tick(&popup.sm);
    }
}

void track_popup_rx(const twai_message_t *msg, can_bus_t rx_bus) {
    if (popup.queue != NULL) {
        sm_rx(&popup.sm, msg, rx_bus);
    }
}

fwd_result_t track_popup_fwd(twai_message_t *msg, can_bus_t fwd_bus) {
    if (popup.queue == NULL) {
        return FWD_PASSTHROUGH;
    }
    return sm_fwd(&popup.sm, msg, fwd_bus);
}

bool track_popup_show(const char *utf8_text) {
    if (popup.queue == NULL) {
        return false;
    }
    track_popup_request_t request = {0};
    if (!encode_text(utf8_text, &request)) {
        return false;
    }
    return xQueueSend(popup.queue, &request, 0) == pdTRUE;
}
