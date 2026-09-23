#pragma once

#include "driver/spi_master.h"
#include "hal/gpio_types.h"
#include "esp_twai.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    struct {
        gpio_num_t int_gpio;                // Active low
        gpio_num_t cs_gpio;
    } io_cfg;
    uint32_t spi_clock_hz;                  // At most 0.85 * oscillator_hz / 2
    uint32_t oscillator_hz;                 // SYSCLK; the PLL is not used
    twai_timing_basic_config_t bit_timing;
    struct {
        uint32_t enable_loopback: 1;        // External loopback: frames also go to the bus
        uint32_t enable_listen_only: 1;
    } flags;
} twai_mcp251xfd_node_config_t;

// Classical CAN TWAI node on an MCP2517FD/MCP2518FD, using Zephyr's driver.
//
// The caller initializes the SPI bus without DMA and installs the GPIO ISR
// service. The node reserves the SPI bus for itself.
//
// There is a single node, initialized on the first call and never freed:
// twai_node_delete() releases it and puts the controller in Sleep mode, and
// the next call wakes it and applies the new bit timing and mode. SPI, pins
// and oscillator come from the first call. If initialization fails, later
// calls fail too until restart.
//
// Callbacks run in the driver's thread rather than an ISR, including
// on_tx_done for frames that twai_node_disable() aborts. Frames retry until
// sent (no one-shot mode). Filter 0 is the only mask filter.
esp_err_t twai_new_node_mcp251xfd(spi_host_device_t host, const twai_mcp251xfd_node_config_t *config,
                                  twai_node_handle_t *node_ret);

#ifdef __cplusplus
}
#endif
