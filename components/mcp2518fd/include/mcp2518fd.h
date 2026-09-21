#ifndef MCP2518FD_H
#define MCP2518FD_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MCP2518FD_MAX_DATA_LEN 64U
#define MCP2518FD_STANDARD_ID_MAX 0x7FFU
#define MCP2518FD_EXTENDED_ID_MAX 0x1FFFFFFFU

typedef struct mcp2518fd_dev_t *mcp2518fd_handle_t;

typedef enum {
    MCP2518FD_MODE_NORMAL_FD = 0,
    MCP2518FD_MODE_SLEEP = 1,
    MCP2518FD_MODE_INTERNAL_LOOPBACK = 2,
    MCP2518FD_MODE_LISTEN_ONLY = 3,
    MCP2518FD_MODE_CONFIGURATION = 4,
    MCP2518FD_MODE_EXTERNAL_LOOPBACK = 5,
    MCP2518FD_MODE_NORMAL_CLASSIC = 6,
    MCP2518FD_MODE_RESTRICTED = 7,
} mcp2518fd_mode_t;

typedef struct {
    uint32_t id;
    uint32_t timestamp;
    uint8_t data_length;
    uint8_t dlc;
    uint8_t filter_hit;
    bool extended_id;
    bool fd_frame;
    bool bit_rate_switch;
    bool remote_frame;
    bool error_state_indicator;
    uint8_t data[MCP2518FD_MAX_DATA_LEN];
} mcp2518fd_frame_t;

typedef struct {
    size_t tx_queue_pending;
    size_t rx_queue_pending;
    bool bus_off;
    bool rx_fifo_pending;
    bool tx_queue_not_full;
} mcp2518fd_status_t;

typedef struct {
    uint32_t c1con;
    uint32_t c1int;
    uint32_t trec;
    uint32_t txqcon;
    uint32_t txqsta;
    uint32_t rx_fifo_status;
    uint32_t bdiag0;
    uint32_t bdiag1;
    uint32_t osc;
    uint32_t devid;
    uint8_t requested_mode;
    uint8_t operating_mode;
    uint8_t tx_error_count;
    uint8_t rx_error_count;
} mcp2518fd_diagnostics_t;

typedef struct {
    spi_host_device_t spi_host;
    gpio_num_t pin_sclk;
    gpio_num_t pin_mosi;
    gpio_num_t pin_miso;
    gpio_num_t pin_cs;
    gpio_num_t pin_int;
    gpio_num_t pin_standby;
    uint32_t spi_clock_hz;
    uint32_t oscillator_hz;
    uint32_t nominal_bitrate;
    uint32_t data_bitrate;
    uint8_t payload_size;
    uint8_t tx_queue_len;
    uint8_t rx_queue_len;
    uint8_t txq_depth;
    uint8_t rx_fifo_depth;
    bool initialize_spi_bus;
    bool enable_can_fd;
    bool enable_brs;
    uint32_t task_stack_size;
    UBaseType_t task_priority;
    mcp2518fd_mode_t mode;
} mcp2518fd_config_t;

#define MCP2518FD_CONFIG_DEFAULT() { \
    .spi_host = SPI2_HOST, \
    .pin_sclk = (gpio_num_t)-1, \
    .pin_mosi = (gpio_num_t)-1, \
    .pin_miso = (gpio_num_t)-1, \
    .pin_cs = (gpio_num_t)-1, \
    .pin_int = (gpio_num_t)-1, \
    .pin_standby = (gpio_num_t)-1, \
    .spi_clock_hz = 10000000U, \
    .oscillator_hz = 20000000U, \
    .nominal_bitrate = 500000U, \
    .data_bitrate = 2000000U, \
    .payload_size = MCP2518FD_MAX_DATA_LEN, \
    .tx_queue_len = 16U, \
    .rx_queue_len = 16U, \
    .txq_depth = 8U, \
    .rx_fifo_depth = 8U, \
    .initialize_spi_bus = true, \
    .enable_can_fd = true, \
    .enable_brs = true, \
    .task_stack_size = 4096U, \
    .task_priority = 10U, \
    .mode = MCP2518FD_MODE_NORMAL_FD, \
}

esp_err_t mcp2518fd_new(const mcp2518fd_config_t *config, mcp2518fd_handle_t *out_handle);
esp_err_t mcp2518fd_delete(mcp2518fd_handle_t handle);

esp_err_t mcp2518fd_start(mcp2518fd_handle_t handle);
esp_err_t mcp2518fd_stop(mcp2518fd_handle_t handle);

esp_err_t mcp2518fd_transmit(mcp2518fd_handle_t handle, const mcp2518fd_frame_t *frame, TickType_t ticks_to_wait);
esp_err_t mcp2518fd_receive(mcp2518fd_handle_t handle, mcp2518fd_frame_t *frame, TickType_t ticks_to_wait);
esp_err_t mcp2518fd_get_status(mcp2518fd_handle_t handle, mcp2518fd_status_t *status);
esp_err_t mcp2518fd_get_diagnostics(mcp2518fd_handle_t handle, mcp2518fd_diagnostics_t *diagnostics);

/* Stop the worker and put the controller into Sleep mode. With low_power=true the
 * device enters Low Power Mode (OSC.LPMEN): SPI is unavailable until the next
 * mcp2518fd_start(), which performs a full reset/re-init. */
esp_err_t mcp2518fd_enter_sleep(mcp2518fd_handle_t handle, bool low_power);

uint8_t mcp2518fd_length_to_dlc(uint8_t data_length);
uint8_t mcp2518fd_dlc_to_length(uint8_t dlc);

#ifdef __cplusplus
}
#endif

#endif