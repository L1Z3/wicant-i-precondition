#include <stddef.h>
#include <string.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "isotp_tx.h"

#define TAG "isotp_tx"

// ********************* ISO-TP protocol constants *********************

// high nibble of byte 0 of all ISO-TP frames contains the frame type
#define ISOTP_PCI_TYPE_MASK 0xF0U
#define ISOTP_PCI_SINGLE_FRAME 0x00U
#define ISOTP_PCI_FIRST_FRAME 0x10U
#define ISOTP_PCI_CONSECUTIVE_FRAME 0x20U
#define ISOTP_PCI_FLOW_CONTROL 0x30U

// low nibble of byte 0 of flow control frames contains the flow status
#define ISOTP_FLOW_STATUS_CONTINUE 0x00U
#define ISOTP_FLOW_STATUS_WAIT 0x01U
#define ISOTP_FLOW_STATUS_OVERFLOW 0x02U

#define ISOTP_SINGLE_FRAME_DATA_SIZE 7U
#define ISOTP_FIRST_FRAME_DATA_SIZE 6U
#define ISOTP_CONSECUTIVE_FRAME_DATA_SIZE 7U

// ********************* state machine declarations *********************

enum {
    EV_START = 1,
};

static const sm_state_t S_IDLE, S_WAIT_FLOW_CONTROL, S_SEND_CONSECUTIVE;

_Static_assert(offsetof(isotp_tx_t, sm) == 0, "sm must be the first isotp_tx_t field");

// ********************* transport and deadline helpers *********************

static isotp_tx_t *owner(sm_t *sm) {
    return (isotp_tx_t *)sm;
}

static void set_deadline(isotp_tx_t *tx, int64_t deadline_us) {
    BaseType_t taken = xSemaphoreTake(tx->deadline_lock, portMAX_DELAY);
    configASSERT(taken == pdTRUE);
    (void)taken;
    tx->next_deadline_us = deadline_us;
    BaseType_t given = xSemaphoreGive(tx->deadline_lock);
    configASSERT(given == pdTRUE);
    (void)given;
}

int64_t isotp_tx_next_deadline_us(isotp_tx_t *tx) {
    BaseType_t taken = xSemaphoreTake(tx->deadline_lock, portMAX_DELAY);
    configASSERT(taken == pdTRUE);
    (void)taken;
    int64_t deadline_us = tx->next_deadline_us;
    BaseType_t given = xSemaphoreGive(tx->deadline_lock);
    configASSERT(given == pdTRUE);
    (void)given;
    return deadline_us;
}

static void wake_worker(isotp_tx_t *tx) {
    if (tx->worker != NULL) {
        xTaskNotifyGive(tx->worker);
    }
}

static void finish(isotp_tx_t *tx, isotp_tx_result_t result) {
    atomic_store_explicit(&tx->result, result, memory_order_release);
    atomic_store_explicit(&tx->busy, false, memory_order_release);
    set_deadline(tx, ISOTP_TX_NO_DEADLINE);
}

static bool send_frame(isotp_tx_t *tx, twai_message_t *frame) {
    frame->identifier = tx->config.tx_id;
    frame->data_length_code = 8U;
    return can_send(tx->config.bus, frame, tx->config.can_send_wait_ticks) == 0;
}

static bool send_single_frame(isotp_tx_t *tx) {
    twai_message_t frame = {0};
    memset(frame.data, tx->config.padding_byte, sizeof(frame.data));
    // payload length in low nibble
    frame.data[0] = ISOTP_PCI_SINGLE_FRAME | (uint8_t)tx->payload_size;
    memcpy(&frame.data[1], tx->payload, tx->payload_size);
    return send_frame(tx, &frame);
}

static bool send_first_frame(isotp_tx_t *tx) {
    twai_message_t frame = {0};
    memset(frame.data, tx->config.padding_byte, sizeof(frame.data));
    // payload length in low nibble of byte 0 + all of byte 1
    frame.data[0] = ISOTP_PCI_FIRST_FRAME
                  | (uint8_t)((tx->payload_size >> 8U) & 0x0FU);
    frame.data[1] = (uint8_t)(tx->payload_size & 0xFFU);
    memcpy(&frame.data[2], tx->payload, ISOTP_FIRST_FRAME_DATA_SIZE);
    if (!send_frame(tx, &frame)) {
        return false;
    }
    tx->payload_offset = ISOTP_FIRST_FRAME_DATA_SIZE;
    tx->sequence_number = 1U;
    tx->wait_frames = 0U;
    return true;
}

static uint32_t decode_separation_time_us(uint8_t encoded) {
    if (encoded <= 0x7FU) {
        // 0x00 to 0x7F => milliseconds
        return (uint32_t)encoded * 1000U;
    }
    if (encoded >= 0xF1U && encoded <= 0xF9U) {
        // 0xF1-0xF9 => microseconds in increments of 100
        return (uint32_t)(encoded - 0xF0U) * 100U;
    }
    // ISO-TP reserves the other values. Treat them as the largest ordinary
    // value rather than sending faster than an unknown receiver requested.
    return 127000U;
}

// ********************* idle state *********************

static bool idle_event(sm_t *sm, sm_event_t ev) {
    if (ev != EV_START) {
        return false;
    }
    isotp_tx_t *tx = owner(sm);
    if (tx->payload_size <= ISOTP_SINGLE_FRAME_DATA_SIZE) {
        finish(tx, send_single_frame(tx)
                   ? ISOTP_TX_RESULT_SUCCESS
                   : ISOTP_TX_RESULT_SEND_ERROR);
        return true;
    }
    if (!send_first_frame(tx)) {
        finish(tx, ISOTP_TX_RESULT_SEND_ERROR);
        return true;
    }
    set_deadline(tx, sm_now(sm) + tx->config.flow_control_timeout_us);
    sm_transition(sm, &S_WAIT_FLOW_CONTROL);
    return true;
}

// ********************* wait-flow-control state *********************

static void wait_flow_control_tick(sm_t *sm) {
    isotp_tx_t *tx = owner(sm);
    if (sm_now(sm) >= isotp_tx_next_deadline_us(tx)) {
        finish(tx, ISOTP_TX_RESULT_TIMEOUT);
        sm_transition(sm, &S_IDLE);
    }
}

static void wait_flow_control_rx(sm_t *sm, const twai_message_t *msg, can_bus_t bus) {
    isotp_tx_t *tx = owner(sm);
    if (bus != tx->config.bus || msg->identifier != tx->config.flow_control_id) {
        return;
    }
    if (msg->data_length_code < 3U
            || (msg->data[0] & ISOTP_PCI_TYPE_MASK) != ISOTP_PCI_FLOW_CONTROL) {
        finish(tx, ISOTP_TX_RESULT_PROTOCOL_ERROR);
        sm_transition(sm, &S_IDLE);
        return;
    }

    switch (msg->data[0] & 0x0FU) {
        case ISOTP_FLOW_STATUS_CONTINUE:
            tx->block_size = msg->data[1];
            tx->block_sent = 0U;
            tx->wait_frames = 0U;
            tx->separation_time_us = decode_separation_time_us(msg->data[2]);
            set_deadline(tx, sm_now(sm));
            sm_transition(sm, &S_SEND_CONSECUTIVE);
            break;
        case ISOTP_FLOW_STATUS_WAIT:
            tx->wait_frames++;
            if (tx->wait_frames > tx->config.max_wait_frames) {
                finish(tx, ISOTP_TX_RESULT_TIMEOUT);
                sm_transition(sm, &S_IDLE);
            } else {
                set_deadline(tx, sm_now(sm) + tx->config.flow_control_timeout_us);
            }
            break;
        case ISOTP_FLOW_STATUS_OVERFLOW:
            finish(tx, ISOTP_TX_RESULT_OVERFLOW);
            sm_transition(sm, &S_IDLE);
            break;
        default:
            finish(tx, ISOTP_TX_RESULT_PROTOCOL_ERROR);
            sm_transition(sm, &S_IDLE);
            break;
    }
}

// ********************* send-consecutive state *********************

static void send_consecutive_tick(sm_t *sm) {
    isotp_tx_t *tx = owner(sm);
    if (sm_now(sm) < isotp_tx_next_deadline_us(tx)) {
        return;
    }

    size_t remaining = tx->payload_size - tx->payload_offset;
    size_t frame_size = remaining < ISOTP_CONSECUTIVE_FRAME_DATA_SIZE
                      ? remaining : ISOTP_CONSECUTIVE_FRAME_DATA_SIZE;
    twai_message_t frame = {0};
    memset(frame.data, tx->config.padding_byte, sizeof(frame.data));
    frame.data[0] = ISOTP_PCI_CONSECUTIVE_FRAME | (tx->sequence_number & 0x0FU);
    memcpy(&frame.data[1], &tx->payload[tx->payload_offset], frame_size);
    if (!send_frame(tx, &frame)) {
        finish(tx, ISOTP_TX_RESULT_SEND_ERROR);
        sm_transition(sm, &S_IDLE);
        return;
    }

    tx->payload_offset += frame_size;
    tx->sequence_number = (uint8_t)((tx->sequence_number + 1U) & 0x0FU);
    tx->block_sent++;

    if (tx->payload_offset >= tx->payload_size) {
        finish(tx, ISOTP_TX_RESULT_SUCCESS);
        sm_transition(sm, &S_IDLE);
    } else if (tx->block_size != 0U && tx->block_sent >= tx->block_size) {
        tx->wait_frames = 0U;
        set_deadline(tx, sm_now(sm) + tx->config.flow_control_timeout_us);
        sm_transition(sm, &S_WAIT_FLOW_CONTROL);
    } else {
        // TODO(ejones): The STmin is from when the message is enqueued in can_send,
        // rather when it's actually sent, so the separation timing isn't always
        // guaranteed if there's a TX backlog. This isn't worth fixing unless it
        // actively starts causing issues.
        set_deadline(tx, sm_now(sm) + tx->separation_time_us);
    }
}

// ********************* state definitions *********************

static const sm_state_t S_IDLE = {
    .name = "idle",
    .event = idle_event,
};

static const sm_state_t S_WAIT_FLOW_CONTROL = {
    .name = "wait-flow-control",
    .tick = wait_flow_control_tick,
    .rx = wait_flow_control_rx,
};

static const sm_state_t S_SEND_CONSECUTIVE = {
    .name = "send-consecutive",
    .tick = send_consecutive_tick,
};

// ********************* core public API *********************

void isotp_tx_init(isotp_tx_t *tx, const char *tag, const isotp_tx_config_t *config) {
    configASSERT(tx != NULL);
    configASSERT(config != NULL);
    configASSERT(config->tx_id <= 0x7FFU);
    configASSERT(config->flow_control_id <= 0x7FFU);
    memset(tx, 0, sizeof(*tx));
    tx->config = *config;
    tx->request_lock = xSemaphoreCreateMutex();
    tx->deadline_lock = xSemaphoreCreateMutex();
    configASSERT(tx->request_lock != NULL);
    configASSERT(tx->deadline_lock != NULL);
    tx->next_deadline_us = ISOTP_TX_NO_DEADLINE;
    atomic_init(&tx->busy, false);
    atomic_init(&tx->start_pending, false);
    atomic_init(&tx->result, ISOTP_TX_RESULT_IDLE);
    sm_init(&tx->sm, tag, &S_IDLE, NULL);
}

bool isotp_tx_start(isotp_tx_t *tx, const uint8_t *payload, size_t payload_size) {
    if (tx == NULL || payload == NULL || payload_size == 0U
            || payload_size > ISOTP_TX_MAX_PAYLOAD_SIZE) {
        return false;
    }

    BaseType_t taken = xSemaphoreTake(tx->request_lock, portMAX_DELAY);
    configASSERT(taken == pdTRUE);
    (void)taken;
    if (atomic_load_explicit(&tx->busy, memory_order_acquire)) {
        xSemaphoreGive(tx->request_lock);
        return false;
    }
    memcpy(tx->payload, payload, payload_size);
    tx->payload_size = payload_size;
    tx->payload_offset = 0U;
    atomic_store_explicit(&tx->result, ISOTP_TX_RESULT_IN_PROGRESS, memory_order_release);
    atomic_store_explicit(&tx->busy, true, memory_order_release);
    atomic_store_explicit(&tx->start_pending, true, memory_order_release);
    set_deadline(tx, esp_timer_get_time());
    BaseType_t given = xSemaphoreGive(tx->request_lock);
    configASSERT(given == pdTRUE);
    (void)given;
    wake_worker(tx);
    return true;
}

bool isotp_tx_rx(isotp_tx_t *tx, const twai_message_t *msg, can_bus_t bus) {
    if (tx == NULL || msg == NULL
            || !atomic_load_explicit(&tx->busy, memory_order_acquire)
            || bus != tx->config.bus
            || msg->identifier != tx->config.flow_control_id) {
        return false;
    }
    sm_rx(&tx->sm, msg, bus);
    wake_worker(tx);
    return true;
}

void isotp_tx_tick(isotp_tx_t *tx) {
    if (atomic_exchange_explicit(&tx->start_pending, false, memory_order_acq_rel)) {
        sm_send_event(&tx->sm, EV_START);
    } else {
        sm_tick(&tx->sm);
    }
}

bool isotp_tx_busy(const isotp_tx_t *tx) {
    return tx != NULL && atomic_load_explicit(&tx->busy, memory_order_acquire);
}

isotp_tx_result_t isotp_tx_result(const isotp_tx_t *tx) {
    if (tx == NULL) {
        return ISOTP_TX_RESULT_PROTOCOL_ERROR;
    }
    return (isotp_tx_result_t)atomic_load_explicit(&tx->result, memory_order_acquire);
}

// ********************* deadline-driven worker *********************

static TickType_t wait_until_deadline(isotp_tx_t *tx) {
    int64_t deadline_us = isotp_tx_next_deadline_us(tx);
    if (deadline_us == ISOTP_TX_NO_DEADLINE) {
        return portMAX_DELAY;
    }
    int64_t delay_us = deadline_us - esp_timer_get_time();
    if (delay_us <= 0) {
        return 0;
    }
    uint64_t delay_ms = ((uint64_t)delay_us + 999U) / 1000U;  // round up
    TickType_t ticks = pdMS_TO_TICKS(delay_ms);
    return ticks == 0 ? 1 : ticks;
}

static void isotp_tx_worker(void *arg) {
    isotp_tx_t *tx = arg;
    while (1) {
        ulTaskNotifyTake(pdTRUE, wait_until_deadline(tx));
        isotp_tx_tick(tx);
    }
}

bool isotp_tx_start_worker(isotp_tx_t *tx, const char *name,
                           uint32_t stack_size, UBaseType_t priority) {
    if (tx == NULL || name == NULL || tx->worker != NULL) {
        return false;
    }
    BaseType_t created = xTaskCreate(isotp_tx_worker, name, stack_size, tx,
                                    priority, &tx->worker);
    if (created != pdPASS) {
        tx->worker = NULL;
        return false;
    }
    wake_worker(tx);
    return true;
}
