#pragma once

#include "driver/spi_master.h"
#include "hal/gpio_types.h"
#include "esp_twai.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    struct {
        gpio_num_t int_gpio;
        gpio_num_t cs_gpio;
    } io_cfg;
    uint32_t spi_clock_hz;
    uint32_t oscillator_hz;
    twai_timing_basic_config_t bit_timing;
    int fail_retry_cnt;             // -1: unlimited retry; 0: one shot
    uint32_t timestamp_resolution_hz; // Software RX timestamp; 0 disables it
    uint32_t tx_queue_depth;        // Total accepted frames, 1..32
    struct {
        uint32_t enable_loopback: 1;
        uint32_t enable_listen_only: 1;
    } flags;
} twai_mcp251xfd_node_config_t;

// The caller initializes the SPI bus and installs the GPIO ISR service.
// Supports MCP2518FD, Classical CAN, and no external reset pin.
// All APIs are task-only. Callbacks execute synchronously under the driver's
// lock in task context (worker or disable caller), unlike on-chip TWAI.
// Callbacks must not call node APIs except receive_from_isr() in on_rx_done.
// Ignore their yield return values; normal FreeRTOS task APIs schedule normally.
// Disable completes outstanding frames with is_tx_success=false. No callbacks
// run after disable returns. Delete joins the worker before freeing resources;
// callers must serialize deletion against other API calls (WiCAN's node_lock).
// Delete also attempts controller LPM, logging any failure without blocking
// resource cleanup, and holds CS high until the next creation. Recreating the
// node wakes the controller at 1 MHz and restores all configuration/RAM.
// Bus-off and controller faults stop the node; recreate it to recover.
esp_err_t twai_new_node_mcp251xfd(spi_host_device_t bus,
                                const twai_mcp251xfd_node_config_t *config,
                                twai_node_handle_t *node_ret);

#ifdef __cplusplus
}
#endif
