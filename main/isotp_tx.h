#ifndef __ISOTP_TX_H__
#define __ISOTP_TX_H__

#include <stdbool.h>
#include <stddef.h>
#include <stdatomic.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "can.h"
#include "hsm.h"

// ISO-TP transmitter for classic CAN using 11-bit IDs and normal
// physical addressing (no address byte in the CAN payload).
// One transfer may be active per instance.
#define ISOTP_TX_MAX_PAYLOAD_SIZE 4095U
#define ISOTP_TX_NO_DEADLINE INT64_MAX

typedef enum {
    ISOTP_TX_RESULT_IDLE = 0,
    ISOTP_TX_RESULT_IN_PROGRESS,
    ISOTP_TX_RESULT_SUCCESS,
    ISOTP_TX_RESULT_TIMEOUT,
    ISOTP_TX_RESULT_OVERFLOW,
    ISOTP_TX_RESULT_PROTOCOL_ERROR,
    ISOTP_TX_RESULT_SEND_ERROR,
} isotp_tx_result_t;

typedef struct {
    can_bus_t bus;
    uint32_t tx_id;
    uint32_t flow_control_id;
    uint32_t flow_control_timeout_us;
    TickType_t can_send_wait_ticks;
    uint8_t max_wait_frames;
    uint8_t padding_byte;
} isotp_tx_config_t;

typedef struct {
    sm_t sm;  // must be first member
    isotp_tx_config_t config;
    SemaphoreHandle_t request_lock;
    SemaphoreHandle_t deadline_lock;
    TaskHandle_t worker;
    atomic_bool busy;
    atomic_bool start_pending;
    atomic_int result;
    int64_t next_deadline_us;
    size_t payload_size;
    size_t payload_offset;
    uint32_t separation_time_us;
    uint8_t block_size;
    uint8_t block_sent;
    uint8_t sequence_number;
    uint8_t wait_frames;
    uint8_t payload[ISOTP_TX_MAX_PAYLOAD_SIZE];
} isotp_tx_t;

// Initialize an instance. config and tag are copied/referenced respectively;
// tag must remain valid for the instance lifetime.
void isotp_tx_init(isotp_tx_t *tx, const char *tag, const isotp_tx_config_t *config);

// Start the deadline-driven worker. This is optional for tests or callers that
// invoke isotp_tx_tick themselves. Returns false if already started or if task
// creation fails.
bool isotp_tx_start_worker(isotp_tx_t *tx, const char *name,
                           uint32_t stack_size, UBaseType_t priority);

// Queue a transfer for the worker. Returns false for invalid input or while a
// previous request is pending/active. Payloads are copied, and so only need
// to last until this function returns. A true return means the payload was
// copied, not that it was transmitted successfully.
bool isotp_tx_start(isotp_tx_t *tx, const uint8_t *payload, size_t payload_size);

// Deliver a received CAN frame. Returns true when it belongs to the active
// transfer's flow-control channel, including malformed flow-control frames.
bool isotp_tx_rx(isotp_tx_t *tx, const twai_message_t *msg, can_bus_t bus);

// Process due work or a pending start. The worker calls this automatically.
void isotp_tx_tick(isotp_tx_t *tx);

// Absolute esp_timer deadline for the next useful tick, or
// ISOTP_TX_NO_DEADLINE while idle.
int64_t isotp_tx_next_deadline_us(isotp_tx_t *tx);
bool isotp_tx_busy(const isotp_tx_t *tx);
isotp_tx_result_t isotp_tx_result(const isotp_tx_t *tx);

#endif
