#include "esp_twai_mcp251xfd.h"
#include "fake_platform.h"
#include <pthread.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>

static twai_frame_t frames[33];
static uint8_t payloads[33][8];
static const twai_frame_t *completed[33];
static bool successful[33];
static atomic_uint completion_count, bus_off_count;
static atomic_uint receive_count;
static atomic_bool pause_callback, callback_entered;

static bool on_tx_done(twai_node_handle_t node, const twai_tx_done_event_data_t *event, void *arg)
{
    (void)node; (void)arg;
    assert(!xPortInIsrContext());
    atomic_store(&callback_entered, true);
    while (atomic_load(&pause_callback)) platform_pause_ms(1);
    unsigned index = atomic_load(&completion_count);
    assert(index < 33);
    for (unsigned i = 0; i < index; i++) assert(completed[i] != event->done_tx_frame);
    completed[index] = event->done_tx_frame;
    successful[index] = event->is_tx_success;
    atomic_store(&completion_count, index + 1);
    return false;
}

static bool on_state_change(twai_node_handle_t node, const twai_state_change_event_data_t *event, void *arg)
{
    (void)node; (void)arg;
    assert(!xPortInIsrContext());
    if (event->new_sta == TWAI_ERROR_BUS_OFF) atomic_fetch_add(&bus_off_count, 1);
    return false;
}

static bool on_rx_done(twai_node_handle_t node, const twai_rx_done_event_data_t *event, void *arg)
{
    (void)event; (void)arg;
    assert(!xPortInIsrContext());
    uint8_t data[8] = {0};
    twai_frame_t frame = {.buffer = data, .buffer_len = sizeof(data)};
    assert(node->receive_isr(node, &frame) == ESP_OK);
    assert(frame.header.id == 0x1234567 && frame.header.ide && frame.header.dlc == 8);
    if (frame.header.rtr) assert(frame.buffer_len == 0);
    else {
        assert(frame.buffer_len == 8);
        for (unsigned i = 0; i < 8; i++) assert(data[i] == i);
    }
    assert(node->receive_isr(node, &frame) == ESP_ERR_INVALID_STATE);
    atomic_fetch_add(&receive_count, 1);
    return false;
}

static twai_mcp251xfd_node_config_t config(void)
{
    return (twai_mcp251xfd_node_config_t){.io_cfg = {.int_gpio = 7, .cs_gpio = 18},
        .spi_clock_hz = 10000000, .oscillator_hz = 40000000,
        .bit_timing = {.bitrate = 500000, .sp_permill = 875}, .fail_retry_cnt = -1, .tx_queue_depth = 32};
}

static twai_node_handle_t create_node(void)
{
    platform_reset();
    atomic_store(&completion_count, 0);
    atomic_store(&bus_off_count, 0);
    atomic_store(&receive_count, 0);
    atomic_store(&pause_callback, false);
    atomic_store(&callback_entered, false);
    memset(completed, 0, sizeof(completed));
    memset(successful, 0, sizeof(successful));
    for (unsigned i = 0; i < 33; i++) frames[i] = (twai_frame_t){.header = {.id = 0x700 - i, .dlc = 8}, .buffer = payloads[i], .buffer_len = 8};
    twai_mcp251xfd_node_config_t cfg = config();
    twai_node_handle_t node;
    assert(twai_new_node_mcp251xfd(1, &cfg, &node) == ESP_OK);
    twai_event_callbacks_t callbacks = {.on_tx_done = on_tx_done, .on_state_change = on_state_change, .on_rx_done = on_rx_done};
    assert(node->register_cbs(node, &callbacks, NULL) == ESP_OK);
    assert(node->enable(node) == ESP_OK);
    return node;
}

static void wait_pending(unsigned count)
{
    for (unsigned i = 0; i < 1000 && platform_pending_tx() != count; i++) platform_pause_ms(1);
    assert(platform_pending_tx() == count);
}

static void wait_completions(unsigned count)
{
    for (unsigned i = 0; i < 1000 && atomic_load(&completion_count) != count; i++) platform_pause_ms(1);
    assert(atomic_load(&completion_count) == count);
}

static void test_worker_and_ownership(void)
{
    twai_node_handle_t node = create_node();
    platform_enter_isr(true);
    assert(node->transmit(node, &frames[0], 0) == ESP_ERR_NOT_SUPPORTED);
    platform_enter_isr(false);
    twai_frame_t invalid = frames[0];
    invalid.header.fdf = true;
    assert(node->transmit(node, &invalid, 0) == ESP_ERR_NOT_SUPPORTED);
    invalid = frames[0];
    invalid.buffer = NULL;
    assert(node->transmit(node, &invalid, 0) == ESP_ERR_INVALID_ARG);
    for (unsigned i = 0; i < 32; i++) assert(node->transmit(node, &frames[i], 0) == ESP_OK);
    assert(node->transmit(node, &frames[32], 0) == ESP_ERR_TIMEOUT);
    assert(node->transmit_wait_done(node, 0) == ESP_ERR_TIMEOUT);
    assert(node->del(node) == ESP_ERR_INVALID_STATE);
    for (unsigned batch = 0; batch < 4; batch++) {
        wait_pending(8);
        platform_finish_tx(8);
        wait_completions((batch + 1) * 8);
    }
    assert(node->transmit_wait_done(node, 100) == ESP_OK);
    for (unsigned i = 0; i < 32; i++) assert(completed[i] == &frames[i] && successful[i]);
    platform_receive(0x1234567, false);
    platform_receive(0x1234567, true);
    for (unsigned i = 0; i < 1000 && atomic_load(&receive_count) != 2; i++) platform_pause_ms(1);
    assert(atomic_load(&receive_count) == 2);
    assert(node->receive_isr(node, &frames[0]) == ESP_ERR_INVALID_STATE);
    assert(node->disable(node) == ESP_OK);
    assert(node->del(node) == ESP_OK);
    platform_check_clean();
}

typedef struct { twai_node_handle_t node; esp_err_t result; atomic_bool started, returned; } operation_t;
static void *disable_thread(void *arg)
{
    operation_t *op = arg;
    atomic_store(&op->started, true);
    op->result = op->node->disable(op->node);
    atomic_store(&op->returned, true);
    return NULL;
}
static void *transmit_thread(void *arg)
{
    operation_t *op = arg;
    atomic_store(&op->started, true);
    op->result = op->node->transmit(op->node, &frames[32], -1);
    atomic_store(&op->returned, true);
    return NULL;
}

static void test_disable_waits_for_callback(void)
{
    twai_node_handle_t node = create_node();
    assert(node->transmit(node, &frames[0], 0) == ESP_OK);
    wait_pending(1);
    atomic_store(&pause_callback, true);
    platform_finish_tx(1);
    for (unsigned i = 0; i < 1000 && !atomic_load(&callback_entered); i++) platform_pause_ms(1);
    assert(atomic_load(&callback_entered));
    operation_t op = {.node = node};
    pthread_t thread;
    assert(pthread_create(&thread, NULL, disable_thread, &op) == 0);
    while (!atomic_load(&op.started)) platform_pause_ms(1);
    platform_pause_ms(20);
    assert(!atomic_load(&op.returned)); // Disable cannot race the active callback.
    atomic_store(&pause_callback, false);
    pthread_join(thread, NULL);
    assert(op.result == ESP_OK && atomic_load(&completion_count) == 1);
    assert(node->del(node) == ESP_OK);
    platform_check_clean();
    assert(atomic_load(&completion_count) == 1);
}

static void test_disable_wakes_full_queue_sender(void)
{
    twai_node_handle_t node = create_node();
    for (unsigned i = 0; i < 32; i++) assert(node->transmit(node, &frames[i], 0) == ESP_OK);
    operation_t op = {.node = node};
    pthread_t thread;
    assert(pthread_create(&thread, NULL, transmit_thread, &op) == 0);
    while (!atomic_load(&op.started)) platform_pause_ms(1);
    platform_pause_ms(20);
    assert(!atomic_load(&op.returned));
    assert(node->disable(node) == ESP_OK);
    pthread_join(thread, NULL);
    assert(op.result == ESP_ERR_INVALID_STATE && atomic_load(&completion_count) == 32);
    for (unsigned i = 0; i < 32; i++) assert(completed[i] == &frames[i] && !successful[i]);
    assert(node->enable(node) == ESP_OK);
    assert(node->transmit(node, &frames[32], 0) == ESP_OK);
    wait_pending(1);
    platform_finish_tx(1);
    wait_completions(33);
    assert(node->disable(node) == ESP_OK);
    assert(node->del(node) == ESP_OK);
    platform_check_clean();
}

static void test_fault_and_failed_creation(void)
{
    twai_node_handle_t node = create_node();
    for (unsigned i = 0; i < 32; i++) assert(node->transmit(node, &frames[i], 0) == ESP_OK);
    platform_disconnect();
    wait_completions(32);
    for (unsigned i = 0; i < 1000 && !atomic_load(&bus_off_count); i++) platform_pause_ms(1);
    assert(atomic_load(&bus_off_count) == 1);
    assert(node->transmit(node, &frames[32], 0) == ESP_ERR_INVALID_STATE);
    assert(node->transmit_wait_done(node, 0) == ESP_ERR_INVALID_STATE);
    assert(node->disable(node) != ESP_OK); // An SPI failure still permits cleanup.
    assert(node->del(node) == ESP_OK);
    platform_check_clean();
    platform_reset();
    platform_fail_isr_install();
    twai_mcp251xfd_node_config_t cfg = config();
    node = (void *)1;
    assert(twai_new_node_mcp251xfd(1, &cfg, &node) == ESP_FAIL && node == NULL);
    platform_check_clean(); // Includes joining a worker from failed creation.
}

int main(void)
{
    test_worker_and_ownership();
    test_disable_waits_for_callback();
    test_disable_wakes_full_queue_sender();
    test_fault_and_failed_creation();
    for (unsigned i = 0; i < 16; i++) {
        twai_node_handle_t node = create_node();
        assert(node->disable(node) == ESP_OK);
        assert(node->del(node) == ESP_OK);
        platform_check_clean();
    }
    puts("MCP2518FD adapter: worker/ISR separation, original frame pointers, queue backpressure, concurrent disable, fault cleanup, and cooperative deletion passed");
    return 0;
}
