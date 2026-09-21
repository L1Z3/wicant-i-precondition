#include "mcp2518fd.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#define MCP2518FD_SPI_CMD_RESET 0x0U
#define MCP2518FD_SPI_CMD_WRITE 0x2U
#define MCP2518FD_SPI_CMD_READ 0x3U

#define MCP2518FD_REG_C1CON 0x000U
#define MCP2518FD_REG_C1NBTCFG 0x004U
#define MCP2518FD_REG_C1DBTCFG 0x008U
#define MCP2518FD_REG_C1TDC 0x00CU
#define MCP2518FD_REG_C1TSCON 0x014U
#define MCP2518FD_REG_C1INT 0x01CU
#define MCP2518FD_REG_C1TREC 0x034U
#define MCP2518FD_REG_C1BDIAG0 0x038U
#define MCP2518FD_REG_C1BDIAG1 0x03CU
#define MCP2518FD_REG_C1TXQCON 0x050U
#define MCP2518FD_REG_C1TXQSTA 0x054U
#define MCP2518FD_REG_C1TXQUA 0x058U
#define MCP2518FD_REG_C1FIFOCON(fifo_index) (0x05CU + (((fifo_index) - 1U) * 12U))
#define MCP2518FD_REG_C1FIFOSTA(fifo_index) (0x060U + (((fifo_index) - 1U) * 12U))
#define MCP2518FD_REG_C1FIFOUA(fifo_index) (0x064U + (((fifo_index) - 1U) * 12U))
#define MCP2518FD_REG_C1FLTCON(index) (0x1D0U + ((index) * 4U))
#define MCP2518FD_REG_C1FLTOBJ(index) (0x1F0U + ((index) * 8U))
#define MCP2518FD_REG_C1MASK(index) (0x1F4U + ((index) * 8U))
#define MCP2518FD_REG_OSC 0xE00U
#define MCP2518FD_REG_ECCCON 0xE0CU
#define MCP2518FD_REG_DEVID 0xE14U

#define MCP2518FD_RAM_START 0x400U
#define MCP2518FD_RAM_SIZE 2048U

#define MCP2518FD_OSC_OSCRDY (1UL << 10)
#define MCP2518FD_OSC_LPMEN (1UL << 3)

#define MCP2518FD_C1CON_REQOP_SHIFT 24U
#define MCP2518FD_C1CON_OPMOD_SHIFT 21U
#define MCP2518FD_C1CON_TXQEN (1UL << 20)
#define MCP2518FD_C1CON_STEF (1UL << 19)
#define MCP2518FD_C1CON_RTXAT (1UL << 16)
#define MCP2518FD_C1CON_BRSDIS (1UL << 12)
#define MCP2518FD_C1CON_ISOCRCEN (1UL << 5)

#define MCP2518FD_C1TSCON_TBCEN (1UL << 16)
#define MCP2518FD_C1TSCON_TBCPRE_MASK 0x3FFUL
#define MCP2518FD_TBC_TICK_HZ 1000000U

#define MCP2518FD_C1INT_RXOVIE (1UL << 27)
#define MCP2518FD_C1INT_RXIE (1UL << 17)

#define MCP2518FD_FIFO_PLSIZE_SHIFT 29U
#define MCP2518FD_FIFO_FSIZE_SHIFT 24U
#define MCP2518FD_FIFO_TXPRI_SHIFT 16U
#define MCP2518FD_FIFO_FRESET (1UL << 10)
#define MCP2518FD_FIFO_TXREQ (1UL << 9)
#define MCP2518FD_FIFO_UINC (1UL << 8)
#define MCP2518FD_FIFO_TXEN (1UL << 7)
#define MCP2518FD_FIFO_RXTSEN (1UL << 5)
#define MCP2518FD_FIFO_RXOVIE (1UL << 3)
#define MCP2518FD_FIFO_TFNRFNIE (1UL << 0)

#define MCP2518FD_TXQ_NOT_FULL_IF (1UL << 0)

#define MCP2518FD_RX_FIFO_NOT_EMPTY_IF (1UL << 0)
#define MCP2518FD_RX_FIFO_OVERFLOW_IF (1UL << 3)

#define MCP2518FD_FLTCON_FLTEN0 (1UL << 7)
#define MCP2518FD_FLTCON_F0BP_SHIFT 0U

#define MCP2518FD_TREC_TXBO (1UL << 21)
#define MCP2518FD_TREC_TEC_MASK (0xFFUL << 8)
#define MCP2518FD_TREC_REC_MASK 0xFFUL

#define MCP2518FD_MODE_WAIT_MS 100U
#define MCP2518FD_OSC_WAIT_MS 100U
#define MCP2518FD_WORKER_POLL_MS 10U
#define MCP2518FD_RX_FIFO_INDEX 1U
#define MCP2518FD_TX_SEQUENCE_MASK 0x7FFFFFU
#define MCP2518FD_SPI_TRANSFER_MAX_DATA_LEN (12U + MCP2518FD_MAX_DATA_LEN)

typedef struct {
    uint8_t brp;
    uint8_t tseg1;
    uint8_t tseg2;
    uint8_t sjw;
} mcp2518fd_bit_timing_t;

struct mcp2518fd_dev_t {
    spi_device_handle_t spi_handle;
    spi_host_device_t spi_host;
    gpio_num_t pin_int;
    gpio_num_t pin_standby;
    QueueHandle_t tx_queue;
    QueueHandle_t rx_queue;
    SemaphoreHandle_t spi_lock;
    TaskHandle_t worker_task;
    uint32_t oscillator_hz;
    uint8_t payload_size;
    uint8_t tx_object_size;
    uint8_t rx_object_size;
    uint32_t task_stack_size;
    UBaseType_t task_priority;
    mcp2518fd_mode_t mode;
    bool initialize_spi_bus;
    bool enable_can_fd;
    bool enable_brs;
    bool owns_spi_bus;
    bool worker_running;
    bool gpio_isr_added;
    uint32_t nominal_bitrate;
    uint32_t data_bitrate;
    uint8_t txq_depth;
    uint8_t rx_fifo_depth;
    uint32_t tx_sequence;
};

static const char *TAG = "mcp2518fd";
static const uint8_t s_dlc_to_len[16] = {0U, 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U, 12U, 16U, 20U, 24U, 32U, 48U, 64U};

static inline uint32_t mcp2518fd_le32_load(const uint8_t *buffer)
{
    return ((uint32_t)buffer[0]) |
           ((uint32_t)buffer[1] << 8) |
           ((uint32_t)buffer[2] << 16) |
           ((uint32_t)buffer[3] << 24);
}

static inline void mcp2518fd_le32_store(uint8_t *buffer, uint32_t value)
{
    buffer[0] = (uint8_t)(value & 0xFFU);
    buffer[1] = (uint8_t)((value >> 8) & 0xFFU);
    buffer[2] = (uint8_t)((value >> 16) & 0xFFU);
    buffer[3] = (uint8_t)((value >> 24) & 0xFFU);
}

uint8_t mcp2518fd_dlc_to_length(uint8_t dlc)
{
    return s_dlc_to_len[dlc & 0x0FU];
}

uint8_t mcp2518fd_length_to_dlc(uint8_t data_length)
{
    if (data_length <= 8U) {
        return data_length;
    }

    switch (data_length) {
        case 12U:
            return 9U;
        case 16U:
            return 10U;
        case 20U:
            return 11U;
        case 24U:
            return 12U;
        case 32U:
            return 13U;
        case 48U:
            return 14U;
        case 64U:
            return 15U;
        default:
            return 0xFFU;
    }
}

static esp_err_t mcp2518fd_validate_payload_size(uint8_t payload_size)
{
    switch (payload_size) {
        case 8U:
        case 12U:
        case 16U:
        case 20U:
        case 24U:
        case 32U:
        case 48U:
        case 64U:
            return ESP_OK;
        default:
            return ESP_ERR_INVALID_ARG;
    }
}

static uint8_t mcp2518fd_payload_size_code(uint8_t payload_size)
{
    switch (payload_size) {
        case 8U:
            return 0U;
        case 12U:
            return 1U;
        case 16U:
            return 2U;
        case 20U:
            return 3U;
        case 24U:
            return 4U;
        case 32U:
            return 5U;
        case 48U:
            return 6U;
        case 64U:
        default:
            return 7U;
    }
}

static esp_err_t mcp2518fd_lock(mcp2518fd_handle_t handle)
{
    if (xSemaphoreTake(handle->spi_lock, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    return ESP_OK;
}

static void mcp2518fd_unlock(mcp2518fd_handle_t handle)
{
    xSemaphoreGive(handle->spi_lock);
}

static esp_err_t mcp2518fd_spi_transfer(mcp2518fd_handle_t handle,
                                        uint8_t command,
                                        uint16_t address,
                                        const uint8_t *tx_data,
                                        uint8_t *rx_data,
                                        size_t data_len)
{
    uint8_t tx_buffer[2U + MCP2518FD_SPI_TRANSFER_MAX_DATA_LEN] = {0};
    uint8_t rx_buffer[sizeof(tx_buffer)] = {0};

    if (data_len > MCP2518FD_SPI_TRANSFER_MAX_DATA_LEN || (2U + data_len) > sizeof(tx_buffer)) {
        return ESP_ERR_INVALID_SIZE;
    }

    tx_buffer[0] = (uint8_t)((command << 4) | ((address >> 8) & 0x0FU));
    tx_buffer[1] = (uint8_t)(address & 0xFFU);
    if (tx_data != NULL && data_len > 0U) {
        memcpy(&tx_buffer[2], tx_data, data_len);
    }

    spi_transaction_t transaction = {
        .length = (2U + data_len) * 8U,
        .tx_buffer = tx_buffer,
        .rx_buffer = rx_buffer,
    };

    esp_err_t err = spi_device_transmit(handle->spi_handle, &transaction);
    if (err != ESP_OK) {
        return err;
    }

    if (rx_data != NULL && data_len > 0U) {
        memcpy(rx_data, &rx_buffer[2], data_len);
    }

    return ESP_OK;
}

static esp_err_t mcp2518fd_read_bytes_locked(mcp2518fd_handle_t handle, uint16_t address, uint8_t *buffer, size_t len)
{
    return mcp2518fd_spi_transfer(handle, MCP2518FD_SPI_CMD_READ, address, NULL, buffer, len);
}

static esp_err_t mcp2518fd_write_bytes_locked(mcp2518fd_handle_t handle, uint16_t address, const uint8_t *buffer, size_t len)
{
    return mcp2518fd_spi_transfer(handle, MCP2518FD_SPI_CMD_WRITE, address, buffer, NULL, len);
}

static esp_err_t mcp2518fd_read_reg32(mcp2518fd_handle_t handle, uint16_t address, uint32_t *value)
{
    uint8_t buffer[4] = {0};
    esp_err_t err = mcp2518fd_lock(handle);
    if (err != ESP_OK) {
        return err;
    }

    err = mcp2518fd_read_bytes_locked(handle, address, buffer, sizeof(buffer));
    mcp2518fd_unlock(handle);
    if (err != ESP_OK) {
        return err;
    }

    *value = mcp2518fd_le32_load(buffer);
    return ESP_OK;
}

static esp_err_t mcp2518fd_write_reg32(mcp2518fd_handle_t handle, uint16_t address, uint32_t value)
{
    uint8_t buffer[4];
    mcp2518fd_le32_store(buffer, value);

    esp_err_t err = mcp2518fd_lock(handle);
    if (err != ESP_OK) {
        return err;
    }

    err = mcp2518fd_write_bytes_locked(handle, address, buffer, sizeof(buffer));
    mcp2518fd_unlock(handle);
    return err;
}

static esp_err_t mcp2518fd_update_reg32(mcp2518fd_handle_t handle, uint16_t address, uint32_t mask, uint32_t value)
{
    uint32_t current = 0;
    esp_err_t err = mcp2518fd_read_reg32(handle, address, &current);
    if (err != ESP_OK) {
        return err;
    }

    current = (current & ~mask) | (value & mask);
    return mcp2518fd_write_reg32(handle, address, current);
}

static esp_err_t mcp2518fd_reset_device(mcp2518fd_handle_t handle)
{
    esp_err_t err = mcp2518fd_lock(handle);
    if (err != ESP_OK) {
        return err;
    }

    err = mcp2518fd_spi_transfer(handle, MCP2518FD_SPI_CMD_RESET, 0U, NULL, NULL, 0U);
    mcp2518fd_unlock(handle);
    if (err != ESP_OK) {
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(2U));
    return ESP_OK;
}

static esp_err_t mcp2518fd_wait_reg32(mcp2518fd_handle_t handle,
                                      uint16_t address,
                                      uint32_t mask,
                                      uint32_t expected,
                                      uint32_t timeout_ms)
{
    TickType_t start = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    while (true) {
        uint32_t value = 0;
        esp_err_t err = mcp2518fd_read_reg32(handle, address, &value);
        if (err != ESP_OK) {
            return err;
        }
        if ((value & mask) == expected) {
            return ESP_OK;
        }
        if ((xTaskGetTickCount() - start) >= timeout_ticks) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }
}

static esp_err_t mcp2518fd_wait_osc_ready(mcp2518fd_handle_t handle)
{
    return mcp2518fd_wait_reg32(handle, MCP2518FD_REG_OSC, MCP2518FD_OSC_OSCRDY, MCP2518FD_OSC_OSCRDY, MCP2518FD_OSC_WAIT_MS);
}

static uint16_t mcp2518fd_resolve_ram_address(uint32_t user_address)
{
    return (uint16_t)(MCP2518FD_RAM_START + user_address);
}

static esp_err_t mcp2518fd_enable_ecc(mcp2518fd_handle_t handle)
{
    uint8_t ecccon = 0;

    esp_err_t err = mcp2518fd_lock(handle);
    if (err != ESP_OK) {
        return err;
    }

    err = mcp2518fd_read_bytes_locked(handle, MCP2518FD_REG_ECCCON, &ecccon, sizeof(ecccon));
    if (err == ESP_OK) {
        ecccon |= 0x01U;
        err = mcp2518fd_write_bytes_locked(handle, MCP2518FD_REG_ECCCON, &ecccon, sizeof(ecccon));
    }

    mcp2518fd_unlock(handle);
    return err;
}

static esp_err_t mcp2518fd_init_ram(mcp2518fd_handle_t handle, uint8_t fill_value)
{
    uint8_t ram_chunk[MCP2518FD_SPI_TRANSFER_MAX_DATA_LEN];
    memset(ram_chunk, fill_value, sizeof(ram_chunk));

    esp_err_t err = mcp2518fd_lock(handle);
    if (err != ESP_OK) {
        return err;
    }

    for (uint16_t offset = 0; offset < MCP2518FD_RAM_SIZE; offset += sizeof(ram_chunk)) {
        size_t chunk_len = MCP2518FD_RAM_SIZE - offset;
        if (chunk_len > sizeof(ram_chunk)) {
            chunk_len = sizeof(ram_chunk);
        }

        err = mcp2518fd_write_bytes_locked(handle,
                                           (uint16_t)(MCP2518FD_RAM_START + offset),
                                           ram_chunk,
                                           chunk_len);
        if (err != ESP_OK) {
            break;
        }
    }

    mcp2518fd_unlock(handle);
    return err;
}

static esp_err_t mcp2518fd_request_mode(mcp2518fd_handle_t handle, mcp2518fd_mode_t mode)
{
    uint32_t request = ((uint32_t)mode) << MCP2518FD_C1CON_REQOP_SHIFT;
    uint32_t expected = ((uint32_t)mode) << MCP2518FD_C1CON_OPMOD_SHIFT;

    esp_err_t err = mcp2518fd_update_reg32(handle,
                                           MCP2518FD_REG_C1CON,
                                           0x7UL << MCP2518FD_C1CON_REQOP_SHIFT,
                                           request);
    if (err != ESP_OK) {
        return err;
    }

    return mcp2518fd_wait_reg32(handle,
                                MCP2518FD_REG_C1CON,
                                0x7UL << MCP2518FD_C1CON_OPMOD_SHIFT,
                                expected,
                                MCP2518FD_MODE_WAIT_MS);
}

static esp_err_t mcp2518fd_calc_bit_timing(uint32_t oscillator_hz,
                                           uint32_t bitrate,
                                           uint16_t tseg1_max,
                                           uint16_t tseg2_max,
                                           uint16_t sjw_max,
                                           uint16_t target_sample_point_per_mille,
                                           mcp2518fd_bit_timing_t *timing)
{
    bool found = false;
    uint32_t best_sp_error = UINT32_MAX;

    if (oscillator_hz == 0U || bitrate == 0U || timing == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    for (uint32_t brp_div = 1U; brp_div <= 256U; ++brp_div) {
        uint32_t denominator = brp_div * bitrate;
        if ((oscillator_hz % denominator) != 0U) {
            continue;
        }

        uint32_t total_tq = oscillator_hz / denominator;
        if (total_tq < 4U) {
            continue;
        }

        for (uint32_t tseg2 = 1U; tseg2 <= tseg2_max; ++tseg2) {
            if (total_tq <= (1U + tseg2)) {
                continue;
            }

            uint32_t tseg1 = total_tq - 1U - tseg2;
            if (tseg1 < 1U || tseg1 > tseg1_max) {
                continue;
            }

            uint32_t sample_point = ((1U + tseg1) * 1000U) / total_tq;
            uint32_t sample_point_error = (sample_point > target_sample_point_per_mille)
                                              ? (sample_point - target_sample_point_per_mille)
                                              : (target_sample_point_per_mille - sample_point);

            if (!found || sample_point_error < best_sp_error) {
                timing->brp = (uint8_t)(brp_div - 1U);
                timing->tseg1 = (uint8_t)(tseg1 - 1U);
                timing->tseg2 = (uint8_t)(tseg2 - 1U);
                timing->sjw = (uint8_t)(((tseg2 < sjw_max) ? tseg2 : sjw_max) - 1U);
                best_sp_error = sample_point_error;
                found = true;
            }
        }
    }

    return found ? ESP_OK : ESP_ERR_NOT_SUPPORTED;
}

static uint32_t mcp2518fd_make_nbtcfg(const mcp2518fd_bit_timing_t *timing)
{
    return ((uint32_t)timing->brp << 24) |
           ((uint32_t)timing->tseg1 << 16) |
           ((uint32_t)timing->tseg2 << 8) |
           timing->sjw;
}

static uint32_t mcp2518fd_make_dbtcfg(const mcp2518fd_bit_timing_t *timing)
{
    return ((uint32_t)timing->brp << 24) |
           ((uint32_t)timing->tseg1 << 16) |
           ((uint32_t)timing->tseg2 << 8) |
           timing->sjw;
}

static uint32_t mcp2518fd_make_tdc(bool enable_brs, const mcp2518fd_bit_timing_t *data_timing)
{
    if (!enable_brs) {
        return 0U;
    }

    /* TDCO = data TSEG1 register value + 1 (= actual TSEG1 in TQ).
     * TDCMode = 0x2 (AUTO). */
    uint32_t tdco = (uint32_t)(data_timing->tseg1 + 1U);
    return (0x2UL << 16) | (tdco << 8);
}

static esp_err_t mcp2518fd_configure_filters(mcp2518fd_handle_t handle)
{
    esp_err_t err = mcp2518fd_write_reg32(handle, MCP2518FD_REG_C1FLTCON(0U), 0U);
    err = (err == ESP_OK) ? mcp2518fd_write_reg32(handle, MCP2518FD_REG_C1FLTOBJ(0U), 0U) : err;
    err = (err == ESP_OK) ? mcp2518fd_write_reg32(handle, MCP2518FD_REG_C1MASK(0U), 0U) : err;
    err = (err == ESP_OK)
              ? mcp2518fd_write_reg32(handle,
                                     MCP2518FD_REG_C1FLTCON(0U),
                                     MCP2518FD_FLTCON_FLTEN0 | ((uint32_t)MCP2518FD_RX_FIFO_INDEX << MCP2518FD_FLTCON_F0BP_SHIFT))
              : err;
    return err;
}

static esp_err_t mcp2518fd_configure_controller(mcp2518fd_handle_t handle)
{
    mcp2518fd_bit_timing_t nominal_timing = {0};
    mcp2518fd_bit_timing_t data_timing = {0};

    esp_err_t err = mcp2518fd_calc_bit_timing(handle->oscillator_hz,
                                              handle->nominal_bitrate,
                                              256U,
                                              128U,
                                              128U,
                                              800U,
                                              &nominal_timing);
    if (err != ESP_OK) {
        return err;
    }

    err = mcp2518fd_calc_bit_timing(handle->oscillator_hz,
                                    handle->enable_can_fd ? handle->data_bitrate : handle->nominal_bitrate,
                                    32U,
                                    16U,
                                    16U,
                                    750U,
                                    &data_timing);
    if (err != ESP_OK) {
        return err;
    }

    /* Keep REQOP=Configuration while writing config registers.
     * The mode switch to Normal happens in mcp2518fd_start after this function. */
    uint32_t c1con = MCP2518FD_C1CON_TXQEN | MCP2518FD_C1CON_RTXAT |
                     ((uint32_t)MCP2518FD_MODE_CONFIGURATION << MCP2518FD_C1CON_REQOP_SHIFT);
    if (handle->enable_can_fd) {
        c1con |= MCP2518FD_C1CON_ISOCRCEN;
    }
    if (!handle->enable_brs) {
        c1con |= MCP2518FD_C1CON_BRSDIS;
    }

    uint32_t txqcon = ((uint32_t)mcp2518fd_payload_size_code(handle->payload_size) << MCP2518FD_FIFO_PLSIZE_SHIFT) |
                      ((uint32_t)(handle->txq_depth - 1U) << MCP2518FD_FIFO_FSIZE_SHIFT) |
                      (0x1FUL << MCP2518FD_FIFO_TXPRI_SHIFT) |
                      MCP2518FD_FIFO_TXEN;

    uint32_t fifocon = ((uint32_t)mcp2518fd_payload_size_code(handle->payload_size) << MCP2518FD_FIFO_PLSIZE_SHIFT) |
                       ((uint32_t)(handle->rx_fifo_depth - 1U) << MCP2518FD_FIFO_FSIZE_SHIFT) |
                       MCP2518FD_FIFO_RXTSEN |
                       MCP2518FD_FIFO_RXOVIE |
                       MCP2518FD_FIFO_TFNRFNIE;

    err = mcp2518fd_write_reg32(handle, MCP2518FD_REG_C1CON, c1con);
    err = (err == ESP_OK) ? mcp2518fd_write_reg32(handle, MCP2518FD_REG_C1NBTCFG, mcp2518fd_make_nbtcfg(&nominal_timing)) : err;
    err = (err == ESP_OK) ? mcp2518fd_write_reg32(handle, MCP2518FD_REG_C1DBTCFG, mcp2518fd_make_dbtcfg(&data_timing)) : err;
    err = (err == ESP_OK) ? mcp2518fd_write_reg32(handle, MCP2518FD_REG_C1TDC, mcp2518fd_make_tdc(handle->enable_brs, &data_timing)) : err;
    /* Run the time base counter at 1 MHz so RX timestamps (RXTSEN) are in microseconds. */
    {
        uint32_t tbcpre = (handle->oscillator_hz / MCP2518FD_TBC_TICK_HZ);
        tbcpre = (tbcpre > 0U) ? (tbcpre - 1U) : 0U;
        uint32_t tscon = MCP2518FD_C1TSCON_TBCEN | (tbcpre & MCP2518FD_C1TSCON_TBCPRE_MASK);
        err = (err == ESP_OK) ? mcp2518fd_write_reg32(handle, MCP2518FD_REG_C1TSCON, tscon) : err;
    }
    /* Reset FIFOs first, then configure - FRESET clears PLSIZE/FSIZE fields */
    err = (err == ESP_OK) ? mcp2518fd_write_reg32(handle, MCP2518FD_REG_C1TXQCON, MCP2518FD_FIFO_FRESET) : err;
    err = (err == ESP_OK) ? mcp2518fd_write_reg32(handle, MCP2518FD_REG_C1TXQCON, txqcon) : err;
    err = (err == ESP_OK) ? mcp2518fd_write_reg32(handle, MCP2518FD_REG_C1FIFOCON(MCP2518FD_RX_FIFO_INDEX), MCP2518FD_FIFO_FRESET) : err;
    err = (err == ESP_OK) ? mcp2518fd_write_reg32(handle, MCP2518FD_REG_C1FIFOCON(MCP2518FD_RX_FIFO_INDEX), fifocon) : err;
    err = (err == ESP_OK) ? mcp2518fd_configure_filters(handle) : err;
    err = (err == ESP_OK) ? mcp2518fd_write_reg32(handle, MCP2518FD_REG_C1INT, MCP2518FD_C1INT_RXOVIE | MCP2518FD_C1INT_RXIE) : err;
    return err;
}

static uint32_t mcp2518fd_encode_id_word(const mcp2518fd_frame_t *frame)
{
    if (frame->extended_id) {
        uint32_t id = frame->id & MCP2518FD_EXTENDED_ID_MAX;
        return ((id & 0x3FFFFU) << 11) | ((id >> 18) & 0x7FFU);
    }

    return frame->id & MCP2518FD_STANDARD_ID_MAX;
}

static void mcp2518fd_decode_id_word(uint32_t id_word, bool extended_id, uint32_t *id)
{
    if (extended_id) {
        *id = ((id_word & 0x7FFU) << 18) | ((id_word >> 11) & 0x3FFFFU);
        return;
    }

    *id = id_word & MCP2518FD_STANDARD_ID_MAX;
}

static esp_err_t mcp2518fd_validate_frame_for_tx(mcp2518fd_handle_t handle, const mcp2518fd_frame_t *frame, uint8_t *dlc)
{
    if (frame == NULL || dlc == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (frame->extended_id) {
        if (frame->id > MCP2518FD_EXTENDED_ID_MAX) {
            return ESP_ERR_INVALID_ARG;
        }
    } else if (frame->id > MCP2518FD_STANDARD_ID_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    if (frame->data_length > handle->payload_size) {
        return ESP_ERR_INVALID_SIZE;
    }

    if (!handle->enable_can_fd) {
        if (frame->fd_frame || frame->bit_rate_switch || frame->data_length > 8U) {
            return ESP_ERR_INVALID_ARG;
        }
    }

    if (frame->remote_frame && frame->fd_frame) {
        return ESP_ERR_INVALID_ARG;
    }

    *dlc = mcp2518fd_length_to_dlc(frame->data_length);
    if (*dlc == 0xFFU) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static void mcp2518fd_build_tx_object(mcp2518fd_handle_t handle,
                                      const mcp2518fd_frame_t *frame,
                                      uint8_t *buffer,
                                      uint8_t dlc)
{
    uint32_t control = ((handle->tx_sequence & MCP2518FD_TX_SEQUENCE_MASK) << 9) |
                       ((frame->error_state_indicator ? 1UL : 0UL) << 8) |
                       ((frame->fd_frame ? 1UL : 0UL) << 7) |
                       ((frame->bit_rate_switch ? 1UL : 0UL) << 6) |
                       ((frame->remote_frame ? 1UL : 0UL) << 5) |
                       ((frame->extended_id ? 1UL : 0UL) << 4) |
                       dlc;

    memset(buffer, 0, handle->tx_object_size);
    mcp2518fd_le32_store(&buffer[0], mcp2518fd_encode_id_word(frame));
    mcp2518fd_le32_store(&buffer[4], control);
    if (frame->data_length > 0U) {
        memcpy(&buffer[8], frame->data, frame->data_length);
    }

    handle->tx_sequence = (handle->tx_sequence + 1U) & MCP2518FD_TX_SEQUENCE_MASK;
}

static void mcp2518fd_parse_rx_object(mcp2518fd_handle_t handle, const uint8_t *buffer, mcp2518fd_frame_t *frame)
{
    uint32_t id_word = mcp2518fd_le32_load(&buffer[0]);
    uint32_t control = mcp2518fd_le32_load(&buffer[4]);
    uint8_t dlc = (uint8_t)(control & 0x0FU);
    uint8_t expected_len = mcp2518fd_dlc_to_length(dlc);

    memset(frame, 0, sizeof(*frame));
    frame->extended_id = ((control >> 4) & 0x1U) != 0U;
    frame->remote_frame = ((control >> 5) & 0x1U) != 0U;
    frame->bit_rate_switch = ((control >> 6) & 0x1U) != 0U;
    frame->fd_frame = ((control >> 7) & 0x1U) != 0U;
    frame->error_state_indicator = ((control >> 8) & 0x1U) != 0U;
    frame->filter_hit = (uint8_t)((control >> 11) & 0x1FU);
    frame->timestamp = mcp2518fd_le32_load(&buffer[8]);
    frame->dlc = dlc;
    frame->data_length = (expected_len > handle->payload_size) ? handle->payload_size : expected_len;
    mcp2518fd_decode_id_word(id_word, frame->extended_id, &frame->id);

    if (frame->data_length > 0U) {
        memcpy(frame->data, &buffer[12], frame->data_length);
    }
}

static esp_err_t mcp2518fd_service_tx_queue(mcp2518fd_handle_t handle)
{
    while (uxQueueMessagesWaiting(handle->tx_queue) > 0U) {
        uint32_t txq_control = 0;
        esp_err_t err = mcp2518fd_read_reg32(handle, MCP2518FD_REG_C1TXQCON, &txq_control);
        if (err != ESP_OK) {
            return err;
        }

        if ((txq_control & MCP2518FD_FIFO_TXREQ) != 0U) {
            break;
        }

        mcp2518fd_frame_t frame = {0};
        if (xQueueReceive(handle->tx_queue, &frame, 0) != pdTRUE) {
            break;
        }

        uint8_t dlc = 0;
        err = mcp2518fd_validate_frame_for_tx(handle, &frame, &dlc);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Dropping invalid TX frame: %s", esp_err_to_name(err));
            continue;
        }

        uint32_t txq_address = 0;
        err = mcp2518fd_read_reg32(handle, MCP2518FD_REG_C1TXQUA, &txq_address);
        if (err != ESP_OK) {
            return err;
        }

        uint8_t tx_object[8U + MCP2518FD_MAX_DATA_LEN] = {0};
        mcp2518fd_build_tx_object(handle, &frame, tx_object, dlc);

        err = mcp2518fd_lock(handle);
        if (err != ESP_OK) {
            return err;
        }

        err = mcp2518fd_write_bytes_locked(handle,
                           mcp2518fd_resolve_ram_address(txq_address),
                           tx_object,
                           handle->tx_object_size);
        if (err == ESP_OK) {
            /* Write UINC and TXREQ to byte 1 of C1TXQCON only, preserving config bits.
             * Byte 1 bit 0 = UINC (reg bit 8), bit 1 = TXREQ (reg bit 9). */
            uint8_t uinc_txreq = 0x03U;
            err = mcp2518fd_write_bytes_locked(handle,
                                               MCP2518FD_REG_C1TXQCON + 1U,
                                               &uinc_txreq,
                                               1U);
        }

        mcp2518fd_unlock(handle);
        if (err != ESP_OK) {
            return err;
        }
    }

    return ESP_OK;
}

static esp_err_t mcp2518fd_service_rx_fifo(mcp2518fd_handle_t handle)
{
    while (true) {
        uint32_t fifo_status = 0;
        esp_err_t err = mcp2518fd_read_reg32(handle, MCP2518FD_REG_C1FIFOSTA(MCP2518FD_RX_FIFO_INDEX), &fifo_status);
        if (err != ESP_OK) {
            return err;
        }

        if ((fifo_status & MCP2518FD_RX_FIFO_OVERFLOW_IF) != 0U) {
            ESP_LOGW(TAG, "RX FIFO overflow detected");
            /* Clear overflow flag by writing 0 to RXOVIF (bit 3) in C1FIFOSTA */
            uint32_t clear_overflow = fifo_status & ~MCP2518FD_RX_FIFO_OVERFLOW_IF;
            mcp2518fd_write_reg32(handle, MCP2518FD_REG_C1FIFOSTA(MCP2518FD_RX_FIFO_INDEX), clear_overflow);
        }

        if ((fifo_status & MCP2518FD_RX_FIFO_NOT_EMPTY_IF) == 0U) {
            break;
        }

        uint32_t fifo_address = 0;
        err = mcp2518fd_read_reg32(handle, MCP2518FD_REG_C1FIFOUA(MCP2518FD_RX_FIFO_INDEX), &fifo_address);
        if (err != ESP_OK) {
            return err;
        }

        uint8_t rx_object[12U + MCP2518FD_MAX_DATA_LEN] = {0};

        err = mcp2518fd_lock(handle);
        if (err != ESP_OK) {
            return err;
        }

        err = mcp2518fd_read_bytes_locked(handle,
                          mcp2518fd_resolve_ram_address(fifo_address),
                          rx_object,
                          handle->rx_object_size);
        if (err == ESP_OK) {
            /* Write UINC to byte 1 of C1FIFOCON only, preserving config bits.
             * Byte 1 bit 0 = UINC (reg bit 8). */
            uint8_t uinc = 0x01U;
            err = mcp2518fd_write_bytes_locked(handle,
                                               MCP2518FD_REG_C1FIFOCON(MCP2518FD_RX_FIFO_INDEX) + 1U,
                                               &uinc,
                                               1U);
        }

        mcp2518fd_unlock(handle);
        if (err != ESP_OK) {
            return err;
        }

        mcp2518fd_frame_t frame;
        mcp2518fd_parse_rx_object(handle, rx_object, &frame);
        if (xQueueSend(handle->rx_queue, &frame, 0) != pdTRUE) {
            ESP_LOGW(TAG, "RX software queue full, dropping frame id=0x%08" PRIX32, frame.id);
        }
    }

    return ESP_OK;
}

static void IRAM_ATTR mcp2518fd_gpio_isr(void *arg)
{
    mcp2518fd_handle_t handle = (mcp2518fd_handle_t)arg;
    BaseType_t high_task_woken = pdFALSE;

    if (handle->worker_task != NULL) {
        vTaskNotifyGiveFromISR(handle->worker_task, &high_task_woken);
    }

    if (high_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void mcp2518fd_worker_task(void *arg)
{
    mcp2518fd_handle_t handle = (mcp2518fd_handle_t)arg;

    while (handle->worker_running) {
        TickType_t wait_ticks = pdMS_TO_TICKS(MCP2518FD_WORKER_POLL_MS);

        (void)ulTaskNotifyTake(pdTRUE, wait_ticks);
        if (!handle->worker_running) {
            break;
        }

        esp_err_t err = mcp2518fd_service_tx_queue(handle);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "TX service error: %s", esp_err_to_name(err));
        }

        err = mcp2518fd_service_rx_fifo(handle);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "RX service error: %s", esp_err_to_name(err));
        }
    }

    handle->worker_task = NULL;
    vTaskDelete(NULL);
}

static esp_err_t mcp2518fd_configure_gpio_input(gpio_num_t pin)
{
    gpio_config_t io_config = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    return gpio_config(&io_config);
}

static esp_err_t mcp2518fd_configure_gpio_output_low(gpio_num_t pin)
{
    gpio_config_t io_config = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&io_config);
    if (err != ESP_OK) {
        return err;
    }

    return gpio_set_level(pin, 0);
}

esp_err_t mcp2518fd_new(const mcp2518fd_config_t *config, mcp2518fd_handle_t *out_handle)
{
    if (config == NULL || out_handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (config->pin_sclk == (gpio_num_t)-1 ||
        config->pin_mosi == (gpio_num_t)-1 ||
        config->pin_miso == (gpio_num_t)-1 ||
        config->pin_cs == (gpio_num_t)-1) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = mcp2518fd_validate_payload_size(config->payload_size);
    if (err != ESP_OK) {
        return err;
    }

    if (config->txq_depth == 0U || config->txq_depth > 32U || config->rx_fifo_depth == 0U || config->rx_fifo_depth > 32U) {
        return ESP_ERR_INVALID_ARG;
    }

    mcp2518fd_handle_t handle = calloc(1, sizeof(*handle));
    if (handle == NULL) {
        return ESP_ERR_NO_MEM;
    }

    handle->spi_host = config->spi_host;
    handle->pin_int = config->pin_int;
    handle->pin_standby = config->pin_standby;
    handle->oscillator_hz = config->oscillator_hz;
    handle->payload_size = config->payload_size;
    handle->tx_object_size = (uint8_t)(8U + config->payload_size);
    handle->rx_object_size = (uint8_t)(12U + config->payload_size);
    handle->task_stack_size = config->task_stack_size;
    handle->task_priority = config->task_priority;
    handle->mode = config->mode;
    handle->initialize_spi_bus = config->initialize_spi_bus;
    handle->enable_can_fd = config->enable_can_fd;
    handle->enable_brs = config->enable_brs;
    handle->nominal_bitrate = config->nominal_bitrate;
    handle->data_bitrate = config->data_bitrate;
    handle->txq_depth = config->txq_depth;
    handle->rx_fifo_depth = config->rx_fifo_depth;

    handle->spi_lock = xSemaphoreCreateMutex();
    handle->tx_queue = xQueueCreate(config->tx_queue_len, sizeof(mcp2518fd_frame_t));
    handle->rx_queue = xQueueCreate(config->rx_queue_len, sizeof(mcp2518fd_frame_t));
    if (handle->spi_lock == NULL || handle->tx_queue == NULL || handle->rx_queue == NULL) {
        err = ESP_ERR_NO_MEM;
        goto fail;
    }

    if (handle->pin_standby != (gpio_num_t)-1) {
        err = mcp2518fd_configure_gpio_output_low(handle->pin_standby);
        if (err != ESP_OK) {
            goto fail;
        }
    }

    if (handle->pin_int != (gpio_num_t)-1) {
        err = mcp2518fd_configure_gpio_input(handle->pin_int);
        if (err != ESP_OK) {
            goto fail;
        }
    }

    if (handle->initialize_spi_bus) {
        spi_bus_config_t bus_config = {
            .mosi_io_num = config->pin_mosi,
            .miso_io_num = config->pin_miso,
            .sclk_io_num = config->pin_sclk,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 2 + handle->rx_object_size,
        };

        err = spi_bus_initialize(config->spi_host, &bus_config, SPI_DMA_CH_AUTO);
        if (err == ESP_OK) {
            handle->owns_spi_bus = true;
        } else if (err != ESP_ERR_INVALID_STATE) {
            goto fail;
        }
    }

    spi_device_interface_config_t dev_config = {
        .clock_speed_hz = (int)config->spi_clock_hz,
        .mode = 0,
        .spics_io_num = config->pin_cs,
        .queue_size = 1,
    };

    err = spi_bus_add_device(config->spi_host, &dev_config, &handle->spi_handle);
    if (err != ESP_OK) {
        goto fail;
    }

    *out_handle = handle;
    return ESP_OK;

fail:
    (void)mcp2518fd_delete(handle);
    return err;
}

esp_err_t mcp2518fd_start(mcp2518fd_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (handle->worker_running) {
        return ESP_ERR_INVALID_STATE;
    }

    xQueueReset(handle->tx_queue);
    xQueueReset(handle->rx_queue);

    esp_err_t err = mcp2518fd_reset_device(handle);
    err = (err == ESP_OK) ? mcp2518fd_wait_osc_ready(handle) : err;
    err = (err == ESP_OK) ? mcp2518fd_enable_ecc(handle) : err;
    err = (err == ESP_OK) ? mcp2518fd_init_ram(handle, 0xFFU) : err;
    err = (err == ESP_OK) ? mcp2518fd_request_mode(handle, MCP2518FD_MODE_CONFIGURATION) : err;
    err = (err == ESP_OK) ? mcp2518fd_configure_controller(handle) : err;
    err = (err == ESP_OK) ? mcp2518fd_request_mode(handle, handle->mode) : err;
    if (err != ESP_OK) {
        return err;
    }

    handle->worker_running = true;
    if (xTaskCreate(mcp2518fd_worker_task,
                    "mcp2518fd",
                    handle->task_stack_size,
                    handle,
                    handle->task_priority,
                    &handle->worker_task) != pdPASS) {
        handle->worker_running = false;
        return ESP_ERR_NO_MEM;
    }

    if (handle->pin_int != (gpio_num_t)-1) {
        err = gpio_install_isr_service(0);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            (void)mcp2518fd_stop(handle);
            return err;
        }

        err = gpio_isr_handler_add(handle->pin_int, mcp2518fd_gpio_isr, handle);
        if (err != ESP_OK) {
            (void)mcp2518fd_stop(handle);
            return err;
        }
        handle->gpio_isr_added = true;
    }

    if (handle->worker_task != NULL) {
        xTaskNotifyGive(handle->worker_task);
    }

    return ESP_OK;
}

esp_err_t mcp2518fd_stop(mcp2518fd_handle_t handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!handle->worker_running) {
        return ESP_OK;
    }

    if (handle->gpio_isr_added && handle->pin_int != (gpio_num_t)-1) {
        (void)gpio_isr_handler_remove(handle->pin_int);
        handle->gpio_isr_added = false;
    }

    handle->worker_running = false;
    if (handle->worker_task != NULL) {
        xTaskNotifyGive(handle->worker_task);
    }

    TickType_t start = xTaskGetTickCount();
    while (handle->worker_task != NULL) {
        if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(100U)) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(pdMS_TO_TICKS(1U));
    }

    return mcp2518fd_request_mode(handle, MCP2518FD_MODE_CONFIGURATION);
}

esp_err_t mcp2518fd_delete(mcp2518fd_handle_t handle)
{
    if (handle == NULL) {
        return ESP_OK;
    }

    if (handle->worker_running) {
        (void)mcp2518fd_stop(handle);
    }

    if (handle->gpio_isr_added && handle->pin_int != (gpio_num_t)-1) {
        (void)gpio_isr_handler_remove(handle->pin_int);
    }

    if (handle->spi_handle != NULL) {
        (void)spi_bus_remove_device(handle->spi_handle);
    }

    if (handle->owns_spi_bus) {
        (void)spi_bus_free(handle->spi_host);
    }

    if (handle->tx_queue != NULL) {
        vQueueDelete(handle->tx_queue);
    }
    if (handle->rx_queue != NULL) {
        vQueueDelete(handle->rx_queue);
    }
    if (handle->spi_lock != NULL) {
        vSemaphoreDelete(handle->spi_lock);
    }

    free(handle);
    return ESP_OK;
}

esp_err_t mcp2518fd_transmit(mcp2518fd_handle_t handle, const mcp2518fd_frame_t *frame, TickType_t ticks_to_wait)
{
    if (handle == NULL || frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!handle->worker_running) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t dlc = 0;
    esp_err_t err = mcp2518fd_validate_frame_for_tx(handle, frame, &dlc);
    if (err != ESP_OK) {
        return err;
    }
    (void)dlc;

    if (xQueueSend(handle->tx_queue, frame, ticks_to_wait) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    if (handle->worker_task != NULL) {
        xTaskNotifyGive(handle->worker_task);
    }

    return ESP_OK;
}

esp_err_t mcp2518fd_receive(mcp2518fd_handle_t handle, mcp2518fd_frame_t *frame, TickType_t ticks_to_wait)
{
    if (handle == NULL || frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (xQueueReceive(handle->rx_queue, frame, ticks_to_wait) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    return ESP_OK;
}

esp_err_t mcp2518fd_get_status(mcp2518fd_handle_t handle, mcp2518fd_status_t *status)
{
    if (handle == NULL || status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t trec = 0;
    uint32_t txqcon = 0;
    uint32_t fifosta = 0;

    esp_err_t err = mcp2518fd_read_reg32(handle, MCP2518FD_REG_C1TREC, &trec);
    err = (err == ESP_OK) ? mcp2518fd_read_reg32(handle, MCP2518FD_REG_C1TXQCON, &txqcon) : err;
    err = (err == ESP_OK) ? mcp2518fd_read_reg32(handle, MCP2518FD_REG_C1FIFOSTA(MCP2518FD_RX_FIFO_INDEX), &fifosta) : err;
    if (err != ESP_OK) {
        return err;
    }

    status->tx_queue_pending = uxQueueMessagesWaiting(handle->tx_queue);
    status->rx_queue_pending = uxQueueMessagesWaiting(handle->rx_queue);
    status->bus_off = (trec & MCP2518FD_TREC_TXBO) != 0U;
    status->rx_fifo_pending = (fifosta & MCP2518FD_RX_FIFO_NOT_EMPTY_IF) != 0U;
    status->tx_queue_not_full = (txqcon & MCP2518FD_FIFO_TXREQ) == 0U;
    return ESP_OK;
}

esp_err_t mcp2518fd_get_diagnostics(mcp2518fd_handle_t handle, mcp2518fd_diagnostics_t *diagnostics)
{
    if (handle == NULL || diagnostics == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(diagnostics, 0, sizeof(*diagnostics));

    esp_err_t err = mcp2518fd_read_reg32(handle, MCP2518FD_REG_C1CON, &diagnostics->c1con);
    err = (err == ESP_OK) ? mcp2518fd_read_reg32(handle, MCP2518FD_REG_C1INT, &diagnostics->c1int) : err;
    err = (err == ESP_OK) ? mcp2518fd_read_reg32(handle, MCP2518FD_REG_C1TREC, &diagnostics->trec) : err;
    err = (err == ESP_OK) ? mcp2518fd_read_reg32(handle, MCP2518FD_REG_C1TXQCON, &diagnostics->txqcon) : err;
    err = (err == ESP_OK) ? mcp2518fd_read_reg32(handle, MCP2518FD_REG_C1TXQSTA, &diagnostics->txqsta) : err;
    err = (err == ESP_OK) ? mcp2518fd_read_reg32(handle, MCP2518FD_REG_C1FIFOSTA(MCP2518FD_RX_FIFO_INDEX), &diagnostics->rx_fifo_status) : err;
    err = (err == ESP_OK) ? mcp2518fd_read_reg32(handle, MCP2518FD_REG_C1BDIAG0, &diagnostics->bdiag0) : err;
    err = (err == ESP_OK) ? mcp2518fd_read_reg32(handle, MCP2518FD_REG_C1BDIAG1, &diagnostics->bdiag1) : err;
    err = (err == ESP_OK) ? mcp2518fd_read_reg32(handle, MCP2518FD_REG_OSC, &diagnostics->osc) : err;
    err = (err == ESP_OK) ? mcp2518fd_read_reg32(handle, MCP2518FD_REG_DEVID, &diagnostics->devid) : err;
    if (err != ESP_OK) {
        return err;
    }

    diagnostics->requested_mode = (uint8_t)((diagnostics->c1con >> MCP2518FD_C1CON_REQOP_SHIFT) & 0x7U);
    diagnostics->operating_mode = (uint8_t)((diagnostics->c1con >> MCP2518FD_C1CON_OPMOD_SHIFT) & 0x7U);
    diagnostics->tx_error_count = (uint8_t)((diagnostics->trec & MCP2518FD_TREC_TEC_MASK) >> 8);
    diagnostics->rx_error_count = (uint8_t)(diagnostics->trec & MCP2518FD_TREC_REC_MASK);
    return ESP_OK;
}
esp_err_t mcp2518fd_enter_sleep(mcp2518fd_handle_t handle, bool low_power)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = mcp2518fd_stop(handle);   /* stops worker, requests Configuration mode */
    if (err != ESP_OK) {
        return err;
    }

    /* Disable all interrupt sources so INT# stays deasserted while asleep. */
    err = mcp2518fd_write_reg32(handle, MCP2518FD_REG_C1INT, 0U);
    if (err != ESP_OK) {
        return err;
    }

    if (low_power) {
        uint8_t osc = 0;
        err = mcp2518fd_lock(handle);
        if (err != ESP_OK) {
            return err;
        }
        err = mcp2518fd_read_bytes_locked(handle, MCP2518FD_REG_OSC, &osc, 1U);
        if (err == ESP_OK) {
            osc |= (uint8_t)MCP2518FD_OSC_LPMEN;
            err = mcp2518fd_write_bytes_locked(handle, MCP2518FD_REG_OSC, &osc, 1U);
        }
        mcp2518fd_unlock(handle);
        if (err != ESP_OK) {
            return err;
        }
    }

    /* Request Sleep mode. In LPM the device stops responding on SPI, so do not wait for OPMOD. */
    err = mcp2518fd_update_reg32(handle,
                                 MCP2518FD_REG_C1CON,
                                 0x7UL << MCP2518FD_C1CON_REQOP_SHIFT,
                                 ((uint32_t)MCP2518FD_MODE_SLEEP) << MCP2518FD_C1CON_REQOP_SHIFT);
    if (err != ESP_OK) {
        return err;
    }
    if (!low_power) {
        return mcp2518fd_wait_reg32(handle, MCP2518FD_REG_C1CON,
                                    0x7UL << MCP2518FD_C1CON_OPMOD_SHIFT,
                                    ((uint32_t)MCP2518FD_MODE_SLEEP) << MCP2518FD_C1CON_OPMOD_SHIFT,
                                    MCP2518FD_MODE_WAIT_MS);
    }
    return ESP_OK;
}
