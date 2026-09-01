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

#define TRACK_POPUP_MEDIA_OFF_TYPE 0x14U
#define TRACK_POPUP_MEDIA_FRAME_ID 0x4CEU

// "Sounds of Nature" as fallback category
#define TRACK_POPUP_FALLBACK_MEDIA_TYPE 0x04U
#define TRACK_POPUP_FALLBACK_ISOTP_TX_ID 0x6E0U
#define TRACK_POPUP_FALLBACK_ISOTP_FLOW_CONTROL_ID 0x6BEU

#define TRACK_POPUP_TARGET_BUS CAN_BUS_0
#define TRACK_POPUP_SOURCE_BUS CAN_BUS_1

#define TRACK_POPUP_TRIGGER_FRAME_COUNT 3U
#define TRACK_POPUP_TRIGGER_SETTLE_US 5000U
#define TRACK_POPUP_TRIGGER_TIMEOUT_US 2000000U
#define TRACK_POPUP_DISPLAY_HOLD_US 5000000U

#define TRACK_POPUP_MAX_TEXT_BYTES \
    (TRACK_POPUP_MAX_TEXT_CODE_UNITS * sizeof(utf16_t))
#define TRACK_POPUP_MAX_PREFIX_BYTES 4U  // for track_popup_show_prefixed
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
    uint16_t tx_id;
    uint16_t flow_control_id;
} track_popup_transport_t;

typedef enum {
    MEDIA_TRANSPORT_UNKNOWN = 0,
    MEDIA_TRANSPORT_APPS,
    MEDIA_TRANSPORT_BLUETOOTH,
    MEDIA_TRANSPORT_PANDORA_IPOD,
    MEDIA_TRANSPORT_RADIO,
    MEDIA_TRANSPORT_SXM,
    MEDIA_TRANSPORT_DAB,
    MEDIA_TRANSPORT_DMB_TV,
    MEDIA_TRANSPORT_USB,
    MEDIA_TRANSPORT_CARPLAY,
    MEDIA_TRANSPORT_ANDROID_AUTO,
    MEDIA_TRANSPORT_COUNT,
} media_transport_family_t;

static const track_popup_transport_t media_transports[MEDIA_TRANSPORT_COUNT] = {
    [MEDIA_TRANSPORT_APPS] = {0x6E0U, 0x6BEU},
    [MEDIA_TRANSPORT_BLUETOOTH] = {0x6D4U, 0x6B4U},
    [MEDIA_TRANSPORT_PANDORA_IPOD] = {0x6E1U, 0x6BFU},
    [MEDIA_TRANSPORT_RADIO] = {0x6DCU, 0x6BBU},
    [MEDIA_TRANSPORT_SXM] = {0x6ECU, 0x6CAU},
    [MEDIA_TRANSPORT_DAB] = {0x6D8U, 0x6B8U},
    [MEDIA_TRANSPORT_DMB_TV] = {0x6D9U, 0x6B9U},
    [MEDIA_TRANSPORT_USB] = {0x6EAU, 0x6C8U},
    [MEDIA_TRANSPORT_CARPLAY] = {0x6D6U, 0x6B6U},
    [MEDIA_TRANSPORT_ANDROID_AUTO] = {0x6D2U, 0x6B2U},
};

// Some obscure media types don't seem to have associated ISO-TP frame IDs,
// and thus cannot be used to display text. Unlisted entries default to
// MEDIA_TRANSPORT_UNKNOWN and use the fallback media type.
static const uint8_t media_transport_map[UINT8_MAX + 1U] = {  // enums are 4 bytes, so use uint8_t as array type
    [0x02U] = MEDIA_TRANSPORT_APPS,
    [0x03U] = MEDIA_TRANSPORT_APPS,
    [0x04U] = MEDIA_TRANSPORT_APPS,
    [0x05U] = MEDIA_TRANSPORT_APPS,
    [0x06U] = MEDIA_TRANSPORT_APPS,
    [0x07U] = MEDIA_TRANSPORT_APPS,
    [0x10U] = MEDIA_TRANSPORT_BLUETOOTH,
    [0x11U] = MEDIA_TRANSPORT_BLUETOOTH,
    [0x1AU] = MEDIA_TRANSPORT_PANDORA_IPOD,
    [0x1BU] = MEDIA_TRANSPORT_PANDORA_IPOD,
    [0x22U] = MEDIA_TRANSPORT_RADIO,
    [0x23U] = MEDIA_TRANSPORT_RADIO,
    [0x28U] = MEDIA_TRANSPORT_RADIO,
    [0x29U] = MEDIA_TRANSPORT_RADIO,
    [0x2AU] = MEDIA_TRANSPORT_APPS,
    [0x2BU] = MEDIA_TRANSPORT_APPS,
    [0x2CU] = MEDIA_TRANSPORT_APPS,
    [0x2DU] = MEDIA_TRANSPORT_APPS,
    [0x2EU] = MEDIA_TRANSPORT_APPS,
    [0x2FU] = MEDIA_TRANSPORT_APPS,
    [0x3EU] = MEDIA_TRANSPORT_APPS,
    [0x3FU] = MEDIA_TRANSPORT_APPS,
    [0x40U] = MEDIA_TRANSPORT_APPS,
    [0x41U] = MEDIA_TRANSPORT_APPS,
    [0x42U] = MEDIA_TRANSPORT_RADIO,
    [0x43U] = MEDIA_TRANSPORT_RADIO,
    [0x44U] = MEDIA_TRANSPORT_RADIO,
    [0x45U] = MEDIA_TRANSPORT_RADIO,
    [0x46U] = MEDIA_TRANSPORT_RADIO,
    [0x47U] = MEDIA_TRANSPORT_RADIO,
    [0x48U] = MEDIA_TRANSPORT_RADIO,
    [0x49U] = MEDIA_TRANSPORT_RADIO,
    [0x4EU] = MEDIA_TRANSPORT_APPS,
    [0x4FU] = MEDIA_TRANSPORT_APPS,
    [0x50U] = MEDIA_TRANSPORT_APPS,
    [0x51U] = MEDIA_TRANSPORT_APPS,
    [0x52U] = MEDIA_TRANSPORT_APPS,
    [0x53U] = MEDIA_TRANSPORT_APPS,
    [0x5CU] = MEDIA_TRANSPORT_APPS,
    [0x5DU] = MEDIA_TRANSPORT_APPS,
    [0x5EU] = MEDIA_TRANSPORT_APPS,
    [0x5FU] = MEDIA_TRANSPORT_APPS,
    [0x60U] = MEDIA_TRANSPORT_APPS,
    [0x61U] = MEDIA_TRANSPORT_APPS,
    [0x62U] = MEDIA_TRANSPORT_SXM,
    [0x63U] = MEDIA_TRANSPORT_SXM,
    [0x6CU] = MEDIA_TRANSPORT_DAB,
    [0x6DU] = MEDIA_TRANSPORT_DAB,
    [0x74U] = MEDIA_TRANSPORT_APPS,
    [0x75U] = MEDIA_TRANSPORT_APPS,
    [0x7AU] = MEDIA_TRANSPORT_APPS,
    [0x7BU] = MEDIA_TRANSPORT_APPS,
    [0x80U] = MEDIA_TRANSPORT_APPS,
    [0x81U] = MEDIA_TRANSPORT_APPS,
    [0x82U] = MEDIA_TRANSPORT_DMB_TV,
    [0x83U] = MEDIA_TRANSPORT_DMB_TV,
    [0x86U] = MEDIA_TRANSPORT_APPS,
    [0x87U] = MEDIA_TRANSPORT_APPS,
    [0x88U] = MEDIA_TRANSPORT_APPS,
    [0x89U] = MEDIA_TRANSPORT_APPS,
    [0x8AU] = MEDIA_TRANSPORT_DMB_TV,
    [0x8BU] = MEDIA_TRANSPORT_DMB_TV,
    [0x8CU] = MEDIA_TRANSPORT_DMB_TV,
    [0x8DU] = MEDIA_TRANSPORT_DMB_TV,
    [0x90U] = MEDIA_TRANSPORT_APPS,
    [0x91U] = MEDIA_TRANSPORT_APPS,
    [0x92U] = MEDIA_TRANSPORT_APPS,
    [0x93U] = MEDIA_TRANSPORT_APPS,
    [0x94U] = MEDIA_TRANSPORT_APPS,
    [0x95U] = MEDIA_TRANSPORT_APPS,
    [0x9AU] = MEDIA_TRANSPORT_APPS,
    [0x9BU] = MEDIA_TRANSPORT_APPS,
    [0x9CU] = MEDIA_TRANSPORT_APPS,
    [0x9DU] = MEDIA_TRANSPORT_APPS,
    [0x9EU] = MEDIA_TRANSPORT_APPS,
    [0x9FU] = MEDIA_TRANSPORT_APPS,
    [0xAAU] = MEDIA_TRANSPORT_APPS,
    [0xABU] = MEDIA_TRANSPORT_APPS,
    [0xACU] = MEDIA_TRANSPORT_APPS,
    [0xADU] = MEDIA_TRANSPORT_APPS,
    [0xAEU] = MEDIA_TRANSPORT_APPS,
    [0xAFU] = MEDIA_TRANSPORT_APPS,
    [0xB0U] = MEDIA_TRANSPORT_APPS,
    [0xB1U] = MEDIA_TRANSPORT_APPS,
    [0xB8U] = MEDIA_TRANSPORT_APPS,
    [0xB9U] = MEDIA_TRANSPORT_APPS,
    [0xBEU] = MEDIA_TRANSPORT_APPS,
    [0xBFU] = MEDIA_TRANSPORT_APPS,
    [0xC2U] = MEDIA_TRANSPORT_PANDORA_IPOD,
    [0xC3U] = MEDIA_TRANSPORT_PANDORA_IPOD,
    [0xC4U] = MEDIA_TRANSPORT_USB,
    [0xC8U] = MEDIA_TRANSPORT_USB,
    [0xC9U] = MEDIA_TRANSPORT_USB,
    [0xCAU] = MEDIA_TRANSPORT_USB,
    [0xDAU] = MEDIA_TRANSPORT_APPS,
    [0xDBU] = MEDIA_TRANSPORT_APPS,
    [0xDCU] = MEDIA_TRANSPORT_CARPLAY,
    [0xDDU] = MEDIA_TRANSPORT_CARPLAY,
    [0xDEU] = MEDIA_TRANSPORT_ANDROID_AUTO,
    [0xDFU] = MEDIA_TRANSPORT_ANDROID_AUTO,
    [0xE0U] = MEDIA_TRANSPORT_APPS,
    [0xEAU] = MEDIA_TRANSPORT_APPS,
    [0xEBU] = MEDIA_TRANSPORT_APPS,
    [0xECU] = MEDIA_TRANSPORT_APPS,
    [0xEDU] = MEDIA_TRANSPORT_APPS,
    [0xF0U] = MEDIA_TRANSPORT_APPS,
    [0xF1U] = MEDIA_TRANSPORT_APPS,
    [0xF2U] = MEDIA_TRANSPORT_APPS,
    [0xF3U] = MEDIA_TRANSPORT_APPS,
};

_Static_assert(MEDIA_TRANSPORT_COUNT <= UINT8_MAX,
               "media transport family must fit in the lookup table");

typedef struct {
    sm_t sm;
    QueueHandle_t queue;
    isotp_tx_t isotp;
    track_popup_request_t pending_request;
    track_popup_transport_t active_transport;
    bool media_type_owned;
} track_popup_t;

typedef struct {
    track_popup_request_t request;
    uint8_t trigger_frames_remaining;
    bool strategy_selected;
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

static bool transport_for_media_type(uint8_t media_type,
                                     track_popup_transport_t *transport) {
    uint8_t family = media_transport_map[media_type];
    if (family == MEDIA_TRANSPORT_UNKNOWN) {
        return false;
    }
    *transport = media_transports[family];
    return true;
}

static bool is_media_frame(const twai_message_t *msg, can_bus_t fwd_bus) {
    return fwd_bus == TRACK_POPUP_TARGET_BUS
        && msg->identifier == TRACK_POPUP_MEDIA_FRAME_ID
        && msg->data_length_code >= 2U;
}

static void set_active_transport(track_popup_t *service,
                                 const track_popup_transport_t *transport) {
    // Strategy selection happens before the ISO-TP transfer starts, 
    // while its worker is idle.
    service->active_transport = *transport;
    service->isotp.config.tx_id = transport->tx_id;
    service->isotp.config.flow_control_id = transport->flow_control_id;
}

static void select_fallback_strategy(track_popup_t *service) {
    const track_popup_transport_t fallback = {
        .tx_id = TRACK_POPUP_FALLBACK_ISOTP_TX_ID,
        .flow_control_id = TRACK_POPUP_FALLBACK_ISOTP_FLOW_CONTROL_ID,
    };
    set_active_transport(service, &fallback);
    service->media_type_owned = true;
    trigger_ctx.strategy_selected = true;
}

static bool select_active_media_strategy(track_popup_t *service,
                                         uint8_t media_type) {
    track_popup_transport_t transport;
    if (media_type == TRACK_POPUP_MEDIA_OFF_TYPE
            || !transport_for_media_type(media_type, &transport)) {
        return false;
    }
    set_active_transport(service, &transport);
    service->media_type_owned = false;
    trigger_ctx.strategy_selected = true;
    ESP_LOGI(TAG, "reusing media type 0x%02X with ISO-TP 0x%03X/0x%03X",
             media_type, transport.tx_id, transport.flow_control_id);
    return true;
}

static fwd_result_t pin_media_type(track_popup_t *service,
                                   twai_message_t *msg, can_bus_t fwd_bus) {
    if (!service->media_type_owned
            || !is_media_frame(msg, fwd_bus)) {
        return FWD_PASSTHROUGH;
    }
    msg->data[0] = TRACK_POPUP_FALLBACK_MEDIA_TYPE;
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
        sm_transition(sm, &S_TRIGGER);
    }
}

// ********************* trigger state *********************

static void trigger_enter(sm_t *sm) {
    track_popup_t *service = owner(sm);
    trigger_ctx.request = service->pending_request;
    trigger_ctx.trigger_frames_remaining = TRACK_POPUP_TRIGGER_FRAME_COUNT;
    trigger_ctx.requested_at_us = sm_now(sm);
    // Defer selection until the next 0x4CE so the media type and its transport
    // pair come from the same current frame we trigger on in either wiring
    // mode.
}

static void trigger_tick(sm_t *sm) {
    track_popup_t *service = owner(sm);
    if (trigger_ctx.strategy_selected
            && trigger_ctx.trigger_frames_remaining == 0U
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
    if (is_media_frame(msg, fwd_bus)) {
        if (!trigger_ctx.strategy_selected
                && !select_active_media_strategy(service, msg->data[0])) {
            select_fallback_strategy(service);
        }

        bool modified = pin_media_type(service, msg, fwd_bus) == FWD_MODIFIED;
        if (trigger_ctx.trigger_frames_remaining > 0U) {
            msg->data[1] = 0x11U;
            trigger_ctx.trigger_frames_remaining--;
            modified = true;
            if (trigger_ctx.trigger_frames_remaining == 0U) {
                trigger_ctx.trigger_forwarded_at_us = sm_now(sm);
            }
        }
        return modified ? FWD_MODIFIED : FWD_PASSTHROUGH;
    }
    if (trigger_ctx.strategy_selected
            && fwd_bus == TRACK_POPUP_TARGET_BUS
            && msg->identifier == service->active_transport.tx_id) {
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
        ESP_LOGI(TAG, "ISO-TP transfer complete");
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
            && msg->identifier == service->active_transport.tx_id) {
        return FWD_BLOCK;
    }
    if (fwd_bus == TRACK_POPUP_SOURCE_BUS
            && msg->identifier == service->active_transport.flow_control_id) {
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
    popup.active_transport = (track_popup_transport_t) {
        .tx_id = TRACK_POPUP_FALLBACK_ISOTP_TX_ID,
        .flow_control_id = TRACK_POPUP_FALLBACK_ISOTP_FLOW_CONTROL_ID,
    };
    popup.queue = xQueueCreate(TRACK_POPUP_QUEUE_DEPTH,
                               sizeof(track_popup_request_t));
    configASSERT(popup.queue != NULL);

    const isotp_tx_config_t config = {
        .bus = TRACK_POPUP_TARGET_BUS,
        .tx_id = popup.active_transport.tx_id,
        .flow_control_id = popup.active_transport.flow_control_id,
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

static bool track_popup_show_prefixed(const char *prefix,
                                      const char *utf8_text) {
    if (popup.queue == NULL || utf8_text == NULL || utf8_text[0] == '\0') {
        return false;
    }

    char prefixed[TRACK_POPUP_MAX_TEXT_UTF8_BYTES
                  + TRACK_POPUP_MAX_PREFIX_BYTES + 1U];
    size_t prefix_size = strlen(prefix);
    size_t text_size = strlen(utf8_text);
    if (prefix_size + text_size >= sizeof(prefixed)) {
        return false;
    }
    memcpy(prefixed, prefix, prefix_size);
    memcpy(prefixed + prefix_size, utf8_text, text_size + 1U);
    return track_popup_show(prefixed);
}

bool track_popup_show_info(const char *utf8_text) {
    return track_popup_show_prefixed("ⓘ ", utf8_text);
}

bool track_popup_show_warning(const char *utf8_text) {
    return track_popup_show_prefixed("⚠ ", utf8_text);
}

bool track_popup_show_error(const char *utf8_text) {
    return track_popup_show_prefixed("‼ ", utf8_text);
}
