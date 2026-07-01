/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "unity.h"
#include "esp_err.h"
#include "esp_rom_sys.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_twai.h"
#include "esp_twai_mcp2515.h"

/* Update these pins to match your MCP2515 wiring. */
static constexpr spi_host_device_t TEST_MCP2515_SPI_HOST = SPI2_HOST;
static constexpr gpio_num_t TEST_MCP2515_SPI_SCLK_GPIO = GPIO_NUM_6;
static constexpr gpio_num_t TEST_MCP2515_SPI_MOSI_GPIO = GPIO_NUM_7;
static constexpr gpio_num_t TEST_MCP2515_SPI_MISO_GPIO = GPIO_NUM_2;
static constexpr gpio_num_t TEST_MCP2515_INT_GPIO = GPIO_NUM_4;
static constexpr gpio_num_t TEST_MCP2515_CS_GPIO = GPIO_NUM_5;
/* Set this to your MCP2515 crystal frequency (common values: 8MHz or 16MHz). */
static constexpr uint32_t TEST_MCP2515_OSC_HZ = 8000000;

static void test_mcp2515_spi_bus_init(void)
{
    spi_bus_config_t bus_cfg = {};
    bus_cfg.sclk_io_num = TEST_MCP2515_SPI_SCLK_GPIO;
    bus_cfg.mosi_io_num = TEST_MCP2515_SPI_MOSI_GPIO;
    bus_cfg.miso_io_num = TEST_MCP2515_SPI_MISO_GPIO;
    bus_cfg.quadwp_io_num = GPIO_NUM_NC;
    bus_cfg.quadhd_io_num = GPIO_NUM_NC;

    TEST_ESP_OK(spi_bus_initialize(TEST_MCP2515_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
}

static void test_mcp2515_spi_bus_deinit(void)
{
    TEST_ESP_OK(spi_bus_free(TEST_MCP2515_SPI_HOST));
}

static twai_mcp2515_node_config_t test_mcp2515_default_config(void)
{
    twai_mcp2515_node_config_t cfg = {};
    cfg.io_cfg.int_gpio = TEST_MCP2515_INT_GPIO;
    cfg.io_cfg.cs_gpio = TEST_MCP2515_CS_GPIO;
    cfg.spi_clock_hz = 5000000;
    cfg.oscillator_hz = TEST_MCP2515_OSC_HZ;
    cfg.bit_timing.bitrate = 125000;
    cfg.bit_timing.sp_permill = 875;
    cfg.tx_queue_depth = 4;
    cfg.fail_retry_cnt = -1; // auto retry
    return cfg;
}

TEST_CASE("twai mcp2515 api basic compile check", "[twai][mcp2515]")
{
    // both the GPIO interrupt and SPI bus are global resources, should be initialized outside of the driver
    TEST_ESP_OK(gpio_install_isr_service(0));
    test_mcp2515_spi_bus_init();

    twai_node_handle_t node = NULL;
    twai_mcp2515_node_config_t cfg = test_mcp2515_default_config();
    TEST_ESP_OK(twai_new_node_mcp2515(TEST_MCP2515_SPI_HOST, &cfg, &node));

    // MCP2515 doesn't support range filter, just check if the function is properly stubbed
    twai_range_filter_config_t range = {};
    TEST_ESP_ERR(ESP_ERR_NOT_SUPPORTED, twai_node_config_range_filter(node, 0, &range));

    TEST_ESP_OK(twai_node_enable(node));

    // MCP2515 doesn't support FD frame, just check if the function is properly stubbed
    twai_frame_t fd_frame = {};
    uint8_t data[8] = {0};
    fd_frame.buffer = data;
    fd_frame.buffer_len = sizeof(data);
    fd_frame.header.id = 0x123;
    fd_frame.header.fdf = 1;
    TEST_ESP_ERR(ESP_ERR_NOT_SUPPORTED, twai_node_transmit(node, &fd_frame, 0));

    TEST_ESP_OK(twai_node_disable(node));
    TEST_ESP_OK(twai_node_delete(node));

    test_mcp2515_spi_bus_deinit();
    gpio_uninstall_isr_service();
}

typedef struct {
    twai_frame_t rx_frame;
    uint8_t rx_buf[TWAI_FRAME_MAX_LEN];
    volatile bool rx_ok;
} test_mcp2515_loopback_ctx_t;

static bool IRAM_ATTR test_mcp2515_rx_done_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    (void)edata;
    test_mcp2515_loopback_ctx_t *ctx = (test_mcp2515_loopback_ctx_t *)user_ctx;
    if (twai_node_receive_from_isr(handle, &ctx->rx_frame) == ESP_OK) {
        ctx->rx_ok = true;
    }
    return false;
}

TEST_CASE("twai mcp2515 loopback tx/rx", "[twai][mcp2515]")
{
    TEST_ESP_OK(gpio_install_isr_service(0));
    test_mcp2515_spi_bus_init();

    twai_node_handle_t node = NULL;
    twai_mcp2515_node_config_t cfg = test_mcp2515_default_config();
    cfg.flags.enable_loopback = 1;
    cfg.timestamp_resolution_hz = 1000; // enable timestamp
    TEST_ESP_OK(twai_new_node_mcp2515(TEST_MCP2515_SPI_HOST, &cfg, &node));

    test_mcp2515_loopback_ctx_t cb_ctx = {};
    cb_ctx.rx_frame.buffer = cb_ctx.rx_buf;
    cb_ctx.rx_frame.buffer_len = sizeof(cb_ctx.rx_buf);
    twai_event_callbacks_t cbs = {};
    cbs.on_rx_done = test_mcp2515_rx_done_cb;
    TEST_ESP_OK(twai_node_register_event_callbacks(node, &cbs, &cb_ctx));
    TEST_ESP_OK(twai_node_enable(node));

    uint8_t tx_data[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    twai_frame_t tx_frame = {};
    tx_frame.header.id = 0x123;
    tx_frame.buffer = tx_data;
    tx_frame.buffer_len = sizeof(tx_data);
    TEST_ESP_OK(twai_node_transmit(node, &tx_frame, 100));

    bool got_rx = false;
    for (int i = 0; i < 200; i++) {
        if (cb_ctx.rx_ok) {
            got_rx = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    TEST_ASSERT_TRUE(got_rx);
    TEST_ASSERT_EQUAL_HEX32(0x123, cb_ctx.rx_frame.header.id);
    TEST_ASSERT_EQUAL_UINT32(sizeof(tx_data), cb_ctx.rx_frame.buffer_len);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(tx_data, cb_ctx.rx_buf, sizeof(tx_data));
    TEST_ASSERT_TRUE(cb_ctx.rx_frame.header.timestamp > 0);

    TEST_ESP_OK(twai_node_disable(node));
    TEST_ESP_OK(twai_node_delete(node));
    test_mcp2515_spi_bus_deinit();
    gpio_uninstall_isr_service();
}

TEST_CASE("twai mcp2515 normal mode with external can node", "[twai][mcp2515][manual]")
{
    /*
     * Manual setup to reproduce with can-utils (Linux host):
     * 1) Bring up CAN adapter at 125 kbit/s:
     *      sudo ip link set can0 down
     *      sudo ip link set can0 type can bitrate 125000
     *      sudo ip link set can0 up
     *
     * 2) Open one terminal to monitor traffic from MCP2515:
     *      candump -L can0,123:7FF
     *
     * 3) Run this test case; it transmits frame ID 0x123 with payload
     *    01 02 03 04 05 06 07 08, and expects TX completion.
     *
     * 4) Send one frame from PC to MCP2515 (another terminal):
     *      cansend can0 321#A1A2A3A4A5A6A7A8
     *
     * 5) This test waits up to 15s for that frame and verifies ID + payload.
     *
     * Hardware note:
     * - Many MCP2515 + TJA1050 modules require 5V supply for stable bus
     *   communication. Symptoms of undervolage can include loopback passing
     *   but normal bus TX/RX failing.
     */
    TEST_ESP_OK(gpio_install_isr_service(0));
    test_mcp2515_spi_bus_init();

    twai_node_handle_t node = NULL;
    twai_mcp2515_node_config_t cfg = test_mcp2515_default_config();
    cfg.timestamp_resolution_hz = 1000;
    cfg.fail_retry_cnt = -1; // auto-retry to tolerate external node bring-up delay
    TEST_ESP_OK(twai_new_node_mcp2515(TEST_MCP2515_SPI_HOST, &cfg, &node));

    test_mcp2515_loopback_ctx_t cb_ctx = {};
    cb_ctx.rx_frame.buffer = cb_ctx.rx_buf;
    cb_ctx.rx_frame.buffer_len = sizeof(cb_ctx.rx_buf);
    twai_event_callbacks_t cbs = {};
    cbs.on_rx_done = test_mcp2515_rx_done_cb;
    TEST_ESP_OK(twai_node_register_event_callbacks(node, &cbs, &cb_ctx));
    TEST_ESP_OK(twai_node_enable(node));

    const uint8_t tx_data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    twai_frame_t tx_frame = {};
    tx_frame.header.id = 0x123;
    tx_frame.buffer = (uint8_t *)tx_data;
    tx_frame.buffer_len = sizeof(tx_data);
    TEST_ESP_OK(twai_node_transmit(node, &tx_frame, 200));
    TEST_ESP_OK(twai_node_transmit_wait_all_done(node, 5000));

    // wait 15s for the frame from external node, which should be sufficient even if the external node is just being brought up
    for (int i = 0; i < 1500; i++) {
        if (cb_ctx.rx_ok) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    TEST_ASSERT_EQUAL_HEX32(0x321, cb_ctx.rx_frame.header.id);
    TEST_ASSERT_EQUAL_UINT32(8, cb_ctx.rx_frame.buffer_len);
    const uint8_t expected_rx[8] = {0xA1, 0xA2, 0xA3, 0xA4, 0xA5, 0xA6, 0xA7, 0xA8};
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected_rx, cb_ctx.rx_buf, sizeof(expected_rx));

    TEST_ESP_OK(twai_node_disable(node));
    TEST_ESP_OK(twai_node_delete(node));
    test_mcp2515_spi_bus_deinit();
    gpio_uninstall_isr_service();
}

typedef struct {
    twai_frame_t rx_frame;
    uint8_t rx_buf[TWAI_FRAME_MAX_LEN];
    volatile uint32_t tx_done_cnt;
    volatile uint32_t rx_done_cnt;
    uint32_t rx_ids[4];
    uint8_t rx_count;
} test_mcp2515_queue_resume_ctx_t;

static bool IRAM_ATTR test_mcp2515_tx_done_count_cb(twai_node_handle_t handle, const twai_tx_done_event_data_t *edata, void *user_ctx)
{
    (void)handle;
    (void)edata;
    test_mcp2515_queue_resume_ctx_t *ctx = (test_mcp2515_queue_resume_ctx_t *)user_ctx;
    ctx->tx_done_cnt += 1;
    return false;
}

static bool IRAM_ATTR test_mcp2515_rx_done_collect_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    (void)edata;
    test_mcp2515_queue_resume_ctx_t *ctx = (test_mcp2515_queue_resume_ctx_t *)user_ctx;
    if (twai_node_receive_from_isr(handle, &ctx->rx_frame) == ESP_OK) {
        if (ctx->rx_count < (sizeof(ctx->rx_ids) / sizeof(ctx->rx_ids[0]))) {
            ctx->rx_ids[ctx->rx_count] = ctx->rx_frame.header.id;
            ctx->rx_count++;
        }
        ctx->rx_done_cnt += 1;
    }
    return false;
}

TEST_CASE("twai mcp2515 queue resumes across disable and enable", "[twai][mcp2515]")
{
    TEST_ESP_OK(gpio_install_isr_service(0));
    test_mcp2515_spi_bus_init();

    twai_node_handle_t node = NULL;
    twai_mcp2515_node_config_t cfg = test_mcp2515_default_config();
    cfg.flags.enable_loopback = 1;
    cfg.tx_queue_depth = 3;
    TEST_ESP_OK(twai_new_node_mcp2515(TEST_MCP2515_SPI_HOST, &cfg, &node));

    test_mcp2515_queue_resume_ctx_t cb_ctx = {};
    cb_ctx.rx_frame.buffer = cb_ctx.rx_buf;
    cb_ctx.rx_frame.buffer_len = sizeof(cb_ctx.rx_buf);
    twai_event_callbacks_t cbs = {};
    cbs.on_tx_done = test_mcp2515_tx_done_count_cb;
    cbs.on_rx_done = test_mcp2515_rx_done_collect_cb;
    TEST_ESP_OK(twai_node_register_event_callbacks(node, &cbs, &cb_ctx));
    TEST_ESP_OK(twai_node_enable(node));

    uint8_t tx_data0[8] = {0x10, 1, 2, 3, 4, 5, 6, 7};
    uint8_t tx_data1[8] = {0x20, 1, 2, 3, 4, 5, 6, 7};
    uint8_t tx_data2[8] = {0x30, 1, 2, 3, 4, 5, 6, 7};
    twai_frame_t tx0 = {};
    tx0.header.id = 0x111;
    tx0.buffer = tx_data0;
    tx0.buffer_len = sizeof(tx_data0);
    twai_frame_t tx1 = {};
    tx1.header.id = 0x222;
    tx1.buffer = tx_data1;
    tx1.buffer_len = sizeof(tx_data1);
    twai_frame_t tx2 = {};
    tx2.header.id = 0x333;
    tx2.buffer = tx_data2;
    tx2.buffer_len = sizeof(tx_data2);

    /**
     * Simulate blocked TX completion path: no TX done ISR can pull the next frame from SW queue.
     */
    TEST_ESP_OK(gpio_intr_disable(TEST_MCP2515_INT_GPIO));

    TEST_ESP_OK(twai_node_transmit(node, &tx0, 100));
    TEST_ESP_OK(twai_node_transmit(node, &tx1, 100));
    TEST_ESP_OK(twai_node_transmit(node, &tx2, 100));
    vTaskDelay(pdMS_TO_TICKS(10));

    TEST_ESP_OK(twai_node_disable(node));
    TEST_ESP_OK(twai_node_enable(node));

    TEST_ESP_OK(twai_node_transmit_wait_all_done(node, 3000));

    vTaskDelay(pdMS_TO_TICKS(10));

    bool saw_222 = false;
    bool saw_333 = false;
    for (uint8_t j = 0; j < cb_ctx.rx_count; j++) {
        if (cb_ctx.rx_ids[j] == 0x222) {
            saw_222 = true;
        } else if (cb_ctx.rx_ids[j] == 0x333) {
            saw_333 = true;
        }
    }

    /* First frame completion interrupt is intentionally blocked before disable,
     * so only the resumed queued frames should contribute callback counts.
     */
    TEST_ASSERT_EQUAL_UINT32(2, cb_ctx.tx_done_cnt);
    TEST_ASSERT_EQUAL_UINT32(2, cb_ctx.rx_done_cnt);

    /* Queued frames before disable must resume after enable. */
    TEST_ASSERT_TRUE(saw_222);
    TEST_ASSERT_TRUE(saw_333);

    TEST_ESP_OK(twai_node_disable(node));
    TEST_ESP_OK(twai_node_delete(node));
    test_mcp2515_spi_bus_deinit();
    gpio_uninstall_isr_service();
}

TEST_CASE("twai mcp2515 mask filter in loopback", "[twai][mcp2515]")
{
    TEST_ESP_OK(gpio_install_isr_service(0));
    test_mcp2515_spi_bus_init();

    twai_node_handle_t node = NULL;
    twai_mcp2515_node_config_t cfg = test_mcp2515_default_config();
    cfg.flags.enable_loopback = 1;
    TEST_ESP_OK(twai_new_node_mcp2515(TEST_MCP2515_SPI_HOST, &cfg, &node));

    uint32_t ids[2] = {0x111, 0x222};
    twai_mask_filter_config_t mask_cfg = {};
    mask_cfg.id_list = ids;
    mask_cfg.num_of_ids = 2;
    mask_cfg.mask = 0x7FF;
    mask_cfg.is_ext = false;
    mask_cfg.no_fd = true;
    TEST_ESP_OK(twai_node_config_mask_filter(node, 0, &mask_cfg));

    /* Also configure the second MCP2515 mask/filter group (filter_id = 1) to
     * avoid default RXB1 filter state accepting unrelated IDs.
     */
    uint32_t ids_group1[4] = {0x111, 0x222, 0x111, 0x222};
    twai_mask_filter_config_t mask_cfg_group1 = mask_cfg;
    mask_cfg_group1.id_list = ids_group1;
    mask_cfg_group1.num_of_ids = 4;
    TEST_ESP_OK(twai_node_config_mask_filter(node, 1, &mask_cfg_group1));

    test_mcp2515_queue_resume_ctx_t cb_ctx = {};
    cb_ctx.rx_frame.buffer = cb_ctx.rx_buf;
    cb_ctx.rx_frame.buffer_len = sizeof(cb_ctx.rx_buf);
    twai_event_callbacks_t cbs = {};
    cbs.on_rx_done = test_mcp2515_rx_done_collect_cb;
    TEST_ESP_OK(twai_node_register_event_callbacks(node, &cbs, &cb_ctx));
    TEST_ESP_OK(twai_node_enable(node));

    uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    twai_frame_t tx = {};
    tx.buffer = data;
    tx.buffer_len = sizeof(data);

    uint32_t test_ids[] = {0x111, 0x222, 0x333};
    for (size_t i = 0; i < (sizeof(test_ids) / sizeof(test_ids[0])); i++) {
        tx.header.id = test_ids[i];
        TEST_ESP_OK(twai_node_transmit(node, &tx, 100));
        TEST_ESP_OK(twai_node_transmit_wait_all_done(node, 3000));
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    bool saw_111 = false;
    bool saw_222 = false;
    bool saw_333 = false;
    for (uint8_t i = 0; i < cb_ctx.rx_count; i++) {
        if (cb_ctx.rx_ids[i] == 0x111) {
            saw_111 = true;
        } else if (cb_ctx.rx_ids[i] == 0x222) {
            saw_222 = true;
        } else if (cb_ctx.rx_ids[i] == 0x333) {
            saw_333 = true;
        }
    }

    TEST_ASSERT_TRUE(saw_111);
    TEST_ASSERT_TRUE(saw_222);
    TEST_ASSERT_FALSE(saw_333);

    TEST_ESP_OK(twai_node_disable(node));
    TEST_ESP_OK(twai_node_delete(node));
    test_mcp2515_spi_bus_deinit();
    gpio_uninstall_isr_service();
}
