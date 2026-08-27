// Host-side behavior test for the track popup state machine.
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "test_support.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "can.h"
#include "track_popup.h"

typedef struct {
    can_bus_t bus;
    twai_message_t msg;
} sent_t;

static sent_t sent[32];
static size_t sent_count;

esp_err_t can_send(can_bus_t bus, twai_message_t *message,
                   TickType_t ticks_to_wait) {
    (void)ticks_to_wait;
    if (sent_count < sizeof(sent) / sizeof(sent[0])) {
        sent[sent_count] = (sent_t){ .bus = bus, .msg = *message };
    }
    sent_count++;
    return 0;
}

typedef struct {
    uint8_t *data;
    UBaseType_t length;
    UBaseType_t item_size;
    UBaseType_t head;
    UBaseType_t count;
} fake_queue_t;

QueueHandle_t xQueueCreate(UBaseType_t length, UBaseType_t item_size) {
    fake_queue_t *queue = calloc(1, sizeof(*queue));
    queue->data = calloc(length, item_size);
    queue->length = length;
    queue->item_size = item_size;
    return queue;
}

BaseType_t xQueueSend(QueueHandle_t handle, const void *item,
                      TickType_t ticks_to_wait) {
    (void)ticks_to_wait;
    fake_queue_t *queue = handle;
    if (queue->count == queue->length) {
        return pdFALSE;
    }
    UBaseType_t tail = (queue->head + queue->count) % queue->length;
    memcpy(queue->data + tail * queue->item_size, item, queue->item_size);
    queue->count++;
    return pdTRUE;
}

BaseType_t xQueueReceive(QueueHandle_t handle, void *item,
                         TickType_t ticks_to_wait) {
    (void)ticks_to_wait;
    fake_queue_t *queue = handle;
    if (queue->count == 0U) {
        return pdFALSE;
    }
    memcpy(item, queue->data + queue->head * queue->item_size,
           queue->item_size);
    queue->head = (queue->head + 1U) % queue->length;
    queue->count--;
    return pdTRUE;
}

// Include the implementation so this focused test can inspect its HSM states
// and contexts. ISO-TP remains linked as a separate production source.
#include "track_popup.c"

static void expect_state(const char *name) {
    CHECK_MSG(strcmp(popup.sm.current->name, name) == 0,
              "expected state %s, got %s", name, popup.sm.current->name);
}

static fwd_result_t fwd(uint32_t id, can_bus_t bus, twai_message_t *out) {
    twai_message_t msg = {0};
    msg.identifier = id;
    msg.data_length_code = 8U;
    fwd_result_t result = track_popup_fwd(&msg, bus);
    if (out != NULL) {
        *out = msg;
    }
    return result;
}

static void rx(uint32_t id, const uint8_t data[8], can_bus_t bus) {
    twai_message_t msg = {0};
    msg.identifier = id;
    msg.data_length_code = 8U;
    memcpy(msg.data, data, sizeof(msg.data));
    track_popup_rx(&msg, bus);
}

static void test_before_init(void) {
    CHECK(!track_popup_show("not initialized"));
    twai_message_t media;
    CHECK(fwd(TRACK_POPUP_MEDIA_FRAME_ID, TRACK_POPUP_TARGET_BUS, &media)
          == FWD_PASSTHROUGH);
}

static void test_popup_flow(void) {
    track_popup_init();
    expect_state("idle");

    // Do not change the user's media category until a popup has been queued.
    twai_message_t media = {0};
    media.identifier = TRACK_POPUP_MEDIA_FRAME_ID;
    media.data_length_code = 8U;
    media.data[0] = 0x14U;
    media.data[1] = 0x01U;
    CHECK(track_popup_fwd(&media, TRACK_POPUP_TARGET_BUS) == FWD_PASSTHROUGH);
    CHECK(media.data[0] == 0x14U);

    CHECK(!track_popup_show(NULL));
    CHECK(!track_popup_show(""));
    track_popup_request_t replacement = {0};
    CHECK(encode_text("\xC0\x80", &replacement)); // overlong UTF-8 NUL
    const uint8_t expected_replacement[] = {0xFDU, 0xFFU};
    CHECK(replacement.size == sizeof(expected_replacement));
    CHECK(memcmp(replacement.data, expected_replacement,
                 sizeof(expected_replacement)) == 0);
    char max_length[TRACK_POPUP_MAX_TEXT_CHARACTERS + 1U];
    memset(max_length, 'x', sizeof(max_length) - 1U);
    max_length[sizeof(max_length) - 1U] = '\0';
    track_popup_request_t max_length_request = {0};
    CHECK(encode_text(max_length, &max_length_request));
    CHECK(max_length_request.size == TRACK_POPUP_MAX_TEXT_BYTES);

    char oversized[TRACK_POPUP_MAX_TEXT_CHARACTERS + 2U];
    memset(oversized, 'x', sizeof(oversized) - 1U);
    oversized[sizeof(oversized) - 1U] = '\0';
    CHECK(!encode_text(oversized, &max_length_request));

    // H, i, space, U+1F30D EARTH GLOBE EUROPE-AFRICA. The non-BMP codepoint
    // must become the UTF-16 surrogate pair D83C DF0D.
    CHECK(track_popup_show("Hi \xF0\x9F\x8C\x8D"));
    track_popup_tick();
    expect_state("trigger");
    CHECK(!trigger_ctx.strategy_selected);
    CHECK(trigger_ctx.trigger_frames_remaining
          == TRACK_POPUP_TRIGGER_FRAME_COUNT);
    CHECK(!popup.media_type_owned);

    media.data[0] = 0x14U;
    media.data[1] = 0x01U;
    CHECK(track_popup_fwd(&media, TRACK_POPUP_TARGET_BUS) == FWD_MODIFIED);
    CHECK(media.data[0] == TRACK_POPUP_FALLBACK_MEDIA_TYPE);
    CHECK(media.data[1] == 0x11U);
    CHECK(trigger_ctx.trigger_frames_remaining
          == TRACK_POPUP_TRIGGER_FRAME_COUNT - 1U);

    media.data[0] = 0x14U;
    media.data[1] = 0x01U;
    CHECK(track_popup_fwd(&media, TRACK_POPUP_TARGET_BUS) == FWD_MODIFIED);
    CHECK(media.data[0] == TRACK_POPUP_FALLBACK_MEDIA_TYPE);
    CHECK(media.data[1] == 0x11U);
    CHECK(trigger_ctx.trigger_frames_remaining
          == TRACK_POPUP_TRIGGER_FRAME_COUNT - 2U);

    media.data[0] = 0x14U;
    media.data[1] = 0x01U;
    CHECK(track_popup_fwd(&media, TRACK_POPUP_TARGET_BUS) == FWD_MODIFIED);
    CHECK(media.data[0] == TRACK_POPUP_FALLBACK_MEDIA_TYPE);
    CHECK(media.data[1] == 0x11U);
    CHECK(trigger_ctx.trigger_frames_remaining == 0U);

    fake_now += TRACK_POPUP_TRIGGER_SETTLE_US - 1U;
    track_popup_tick();
    expect_state("trigger");
    fake_now++;
    track_popup_tick();
    expect_state("sending");
    CHECK(isotp_tx_busy(&popup.isotp));
    CHECK(sent_count == 0U);

    // The fake worker does not run; manually dispatch its due start.
    isotp_tx_tick(&popup.isotp);
    CHECK(sent_count == 1U);
    CHECK(sent[0].bus == TRACK_POPUP_TARGET_BUS);
    CHECK(sent[0].msg.identifier == TRACK_POPUP_FALLBACK_ISOTP_TX_ID);
    const uint8_t expected_first[] = {
        0x10U, 0x0AU, 'H', 0x00U, 'i', 0x00U, ' ', 0x00U,
    };
    CHECK(memcmp(sent[0].msg.data, expected_first, sizeof(expected_first)) == 0);

    CHECK(fwd(TRACK_POPUP_FALLBACK_ISOTP_TX_ID,
              TRACK_POPUP_TARGET_BUS, NULL)
          == FWD_BLOCK);

    uint8_t flow_control[8] = {0x30U, 0x00U, 0x05U};
    rx(TRACK_POPUP_FALLBACK_ISOTP_FLOW_CONTROL_ID, flow_control,
       TRACK_POPUP_TARGET_BUS);
    CHECK(fwd(TRACK_POPUP_FALLBACK_ISOTP_FLOW_CONTROL_ID,
              TRACK_POPUP_SOURCE_BUS, NULL) == FWD_BLOCK);
    isotp_tx_tick(&popup.isotp);
    CHECK(sent_count == 2U);
    const uint8_t expected_consecutive[] = {
        0x21U, 0x3CU, 0xD8U, 0x0DU, 0xDFU, 0x00U, 0x00U, 0x00U,
    };
    CHECK(memcmp(sent[1].msg.data, expected_consecutive,
                 sizeof(expected_consecutive)) == 0);
    CHECK(isotp_tx_result(&popup.isotp) == ISOTP_TX_RESULT_SUCCESS);

    track_popup_tick();
    expect_state("hold");
    CHECK(fwd(TRACK_POPUP_FALLBACK_ISOTP_TX_ID,
              TRACK_POPUP_TARGET_BUS, NULL) == FWD_BLOCK);
    CHECK(fwd(TRACK_POPUP_FALLBACK_ISOTP_FLOW_CONTROL_ID,
              TRACK_POPUP_SOURCE_BUS, NULL) == FWD_BLOCK);

    // Keep the media category pinned while the cluster displays the completed
    // popup; leave the ordinary byte-1 value untouched.
    media.data[0] = 0x14U;
    media.data[1] = 0x01U;
    CHECK(track_popup_fwd(&media, TRACK_POPUP_TARGET_BUS) == FWD_MODIFIED);
    CHECK(media.data[0] == TRACK_POPUP_FALLBACK_MEDIA_TYPE);
    CHECK(media.data[1] == 0x01U);
    CHECK(popup.media_type_owned);

    fake_now += TRACK_POPUP_DISPLAY_HOLD_US - 1U;
    track_popup_tick();
    expect_state("hold");
    CHECK(fwd(TRACK_POPUP_FALLBACK_ISOTP_TX_ID,
              TRACK_POPUP_TARGET_BUS, NULL) == FWD_BLOCK);
    CHECK(fwd(TRACK_POPUP_FALLBACK_ISOTP_FLOW_CONTROL_ID,
              TRACK_POPUP_SOURCE_BUS, NULL) == FWD_BLOCK);

    fake_now++;
    track_popup_tick();
    expect_state("idle");
    CHECK(fwd(TRACK_POPUP_FALLBACK_ISOTP_TX_ID,
              TRACK_POPUP_TARGET_BUS, NULL) == FWD_PASSTHROUGH);
    CHECK(fwd(TRACK_POPUP_FALLBACK_ISOTP_FLOW_CONTROL_ID,
              TRACK_POPUP_SOURCE_BUS, NULL) == FWD_PASSTHROUGH);

    // Release the emulated category when the display hold expires.
    media.data[0] = 0x14U;
    media.data[1] = 0x01U;
    CHECK(track_popup_fwd(&media, TRACK_POPUP_TARGET_BUS) == FWD_PASSTHROUGH);
    CHECK(media.data[0] == 0x14U);
    CHECK(media.data[1] == 0x01U);
    CHECK(!popup.media_type_owned);
}

static void expect_transport(uint8_t media_type, uint16_t tx_id,
                             uint16_t flow_control_id) {
    track_popup_transport_t transport = {0};
    CHECK_MSG(transport_for_media_type(media_type, &transport),
              "media type 0x%02X should be known", media_type);
    CHECK_MSG(transport.tx_id == tx_id,
              "media type 0x%02X TX: expected 0x%03X, got 0x%03X",
              media_type, tx_id, transport.tx_id);
    CHECK_MSG(transport.flow_control_id == flow_control_id,
              "media type 0x%02X FC: expected 0x%03X, got 0x%03X",
              media_type, flow_control_id, transport.flow_control_id);
}

static void test_media_transport_map(void) {
    const size_t map_size =
        sizeof(media_transport_map) / sizeof(media_transport_map[0]);
    CHECK(map_size == UINT8_MAX + 1U);
    size_t known_media_types = 0U;
    for (size_t i = 0U; i < map_size; i++) {
        CHECK(media_transport_map[i] < MEDIA_TRANSPORT_COUNT);
        if (media_transport_map[i] != MEDIA_TRANSPORT_UNKNOWN) {
            known_media_types++;
        }
    }
    CHECK(known_media_types == 109U);

    expect_transport(0x04U, 0x6E0U, 0x6BEU);  // Sounds of Nature
    expect_transport(0x10U, 0x6D4U, 0x6B4U);  // Bluetooth
    expect_transport(0x1AU, 0x6E1U, 0x6BFU);  // Pandora
    expect_transport(0x22U, 0x6DCU, 0x6BBU);  // FM
    expect_transport(0x62U, 0x6ECU, 0x6CAU);  // SXM
    expect_transport(0x6CU, 0x6D8U, 0x6B8U);  // DAB
    expect_transport(0x82U, 0x6D9U, 0x6B9U);  // DMB
    expect_transport(0xC2U, 0x6E1U, 0x6BFU);  // iPod
    expect_transport(0xC4U, 0x6EAU, 0x6C8U);  // USB
    expect_transport(0xDCU, 0x6D6U, 0x6B6U);  // CarPlay
    expect_transport(0xDEU, 0x6D2U, 0x6B2U);  // Android Auto
    expect_transport(0xF3U, 0x6E0U, 0x6BEU);  // last Apps type

    track_popup_transport_t transport;
    CHECK(!transport_for_media_type(TRACK_POPUP_MEDIA_OFF_TYPE, &transport));
    CHECK(!transport_for_media_type(0x12U, &transport));
    CHECK(!transport_for_media_type(0xC5U, &transport));
    CHECK(!transport_for_media_type(0xFFU, &transport));
}

static void test_known_media(void) {
    sent_count = 0U;
    track_popup_init();
    CHECK(track_popup_show("Known"));
    track_popup_tick();
    expect_state("trigger");
    CHECK(!trigger_ctx.strategy_selected);
    CHECK(!popup.media_type_owned);

    twai_message_t media = {0};
    media.identifier = TRACK_POPUP_MEDIA_FRAME_ID;
    media.data_length_code = 8U;
    media.data[0] = 0xDEU;
    media.data[1] = 0x01U;
    CHECK(track_popup_fwd(&media, TRACK_POPUP_TARGET_BUS) == FWD_MODIFIED);
    CHECK(media.data[0] == 0xDEU);
    CHECK(media.data[1] == 0x11U);
    CHECK(trigger_ctx.strategy_selected);
    CHECK(!popup.media_type_owned);
    CHECK(trigger_ctx.trigger_frames_remaining
          == TRACK_POPUP_TRIGGER_FRAME_COUNT - 1U);
    CHECK(popup.active_transport.tx_id == 0x6D2U);
    CHECK(popup.active_transport.flow_control_id == 0x6B2U);

    media.data[0] = 0xDEU;
    media.data[1] = 0x01U;
    CHECK(track_popup_fwd(&media, TRACK_POPUP_TARGET_BUS) == FWD_MODIFIED);
    CHECK(media.data[0] == 0xDEU);
    CHECK(media.data[1] == 0x11U);
    CHECK(trigger_ctx.trigger_frames_remaining
          == TRACK_POPUP_TRIGGER_FRAME_COUNT - 2U);

    media.data[0] = 0xDEU;
    media.data[1] = 0x01U;
    CHECK(track_popup_fwd(&media, TRACK_POPUP_TARGET_BUS) == FWD_MODIFIED);
    CHECK(media.data[0] == 0xDEU);
    CHECK(media.data[1] == 0x11U);
    CHECK(trigger_ctx.trigger_frames_remaining == 0U);

    // Known-media mode stops modifying 0x4CE as soon as its trigger frames
    // have been handled.
    media.data[0] = 0xDEU;
    media.data[1] = 0x01U;
    CHECK(track_popup_fwd(&media, TRACK_POPUP_TARGET_BUS) == FWD_PASSTHROUGH);
    CHECK(media.data[0] == 0xDEU);
    CHECK(media.data[1] == 0x01U);

    fake_now += TRACK_POPUP_TRIGGER_SETTLE_US;
    track_popup_tick();
    expect_state("sending");
    isotp_tx_tick(&popup.isotp);
    CHECK(sent_count == 1U);
    CHECK(sent[0].msg.identifier == 0x6D2U);

    uint8_t flow_control[8] = {0x30U, 0x00U, 0x00U};
    rx(0x6B2U, flow_control, TRACK_POPUP_TARGET_BUS);
    isotp_tx_tick(&popup.isotp);
    CHECK(sent_count == 2U);
    CHECK(sent[1].msg.identifier == 0x6D2U);
    CHECK(isotp_tx_result(&popup.isotp) == ISOTP_TX_RESULT_SUCCESS);
    track_popup_tick();
    expect_state("hold");

    media.data[0] = 0xDEU;
    media.data[1] = 0x01U;
    CHECK(track_popup_fwd(&media, TRACK_POPUP_TARGET_BUS) == FWD_PASSTHROUGH);
    CHECK(media.data[0] == 0xDEU);
    CHECK(media.data[1] == 0x01U);
}

static void expect_fallback(uint8_t media_type) {
    sent_count = 0U;
    track_popup_init();
    CHECK(track_popup_show("Fallback"));
    track_popup_tick();
    expect_state("trigger");

    twai_message_t media = {0};
    media.identifier = TRACK_POPUP_MEDIA_FRAME_ID;
    media.data_length_code = 8U;
    media.data[0] = media_type;
    media.data[1] = 0x01U;
    CHECK(track_popup_fwd(&media, TRACK_POPUP_TARGET_BUS) == FWD_MODIFIED);
    CHECK(trigger_ctx.strategy_selected);
    CHECK(popup.media_type_owned);
    CHECK(media.data[0] == TRACK_POPUP_FALLBACK_MEDIA_TYPE);
    CHECK(media.data[1] == 0x11U);
    CHECK(trigger_ctx.trigger_frames_remaining
          == TRACK_POPUP_TRIGGER_FRAME_COUNT - 1U);
    CHECK(popup.active_transport.tx_id
          == TRACK_POPUP_FALLBACK_ISOTP_TX_ID);
    CHECK(popup.active_transport.flow_control_id
          == TRACK_POPUP_FALLBACK_ISOTP_FLOW_CONTROL_ID);
}

static void test_fallbacks(void) {
    expect_fallback(TRACK_POPUP_MEDIA_OFF_TYPE);
    expect_fallback(0x12U);
}

int main(void) {
    test_before_init();
    test_popup_flow();
    test_media_transport_map();
    test_known_media();
    test_fallbacks();
    return test_report("track popup state machine");
}
