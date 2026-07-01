#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_twai.h"
#include "esp_twai_mcp2515.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hw_config.h"

#define APP_RX_QUEUE_DEPTH               16U
#define APP_TEST_FRAME_PERIOD_MS         1000U
#define APP_TIMESTAMP_RESOLUTION_HZ      1000U
#define APP_TX_QUEUE_DEPTH               8U
#define APP_TX_TIMEOUT_MS                1000
#define APP_INTERNAL_TEST_ID             0x611U
#define APP_EXTERNAL_TEST_ID             0x612U
#define APP_BATTERY_LOG_PERIOD_MS        1000U

#define APP_VBAT_DIVIDER_R_TOP_OHM       62000.0f
#define APP_VBAT_DIVIDER_R_BOTTOM_OHM    6200.0f
#define APP_VBAT_ADC_ATTEN               ADC_ATTEN_DB_6

static const char *TAG = "dual_can_test";

typedef enum {
	CAN_CHANNEL_INTERNAL = 0,
	CAN_CHANNEL_EXTERNAL,
} can_channel_id_t;

typedef struct {
	can_channel_id_t channel;
	uint32_t id;
	size_t len;
	uint64_t timestamp;
	uint8_t data[TWAI_FRAME_MAX_LEN];
} can_rx_event_t;

typedef struct {
	const char *name;
	can_channel_id_t channel_id;
	twai_node_handle_t node;
	QueueHandle_t rx_queue;
} can_channel_ctx_t;

typedef struct {
	QueueHandle_t rx_queue;
	can_channel_ctx_t internal;
	can_channel_ctx_t external;
} dual_can_app_t;

static dual_can_app_t s_app;
static adc_oneshot_unit_handle_t s_vbat_adc_handle;
static adc_cali_handle_t s_vbat_cali_handle;
static bool s_vbat_cali_enabled;

static const char *channel_name(can_channel_id_t channel_id)
{
	return (channel_id == CAN_CHANNEL_INTERNAL) ? "internal" : "external";
}

static bool can_rx_done_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
	(void)edata;
	can_channel_ctx_t *channel = (can_channel_ctx_t *)user_ctx;
	can_rx_event_t rx_event = {
		.channel = channel->channel_id,
	};
	twai_frame_t rx_frame = {
		.buffer = rx_event.data,
		.buffer_len = sizeof(rx_event.data),
	};

	if (twai_node_receive_from_isr(handle, &rx_frame) != ESP_OK) {
		return false;
	}

	rx_event.id = rx_frame.header.id;
	rx_event.len = rx_frame.buffer_len;
	rx_event.timestamp = rx_frame.header.timestamp;

	BaseType_t task_woken = pdFALSE;
	xQueueSendFromISR(channel->rx_queue, &rx_event, &task_woken);
	return (task_woken == pdTRUE);
}

static void format_data_hex(const uint8_t *data, size_t len, char *buffer, size_t buffer_size)
{
	size_t offset = 0;

	if (buffer_size == 0) {
		return;
	}

	buffer[0] = '\0';
	for (size_t index = 0; index < len && offset + 3 < buffer_size; index++) {
		int written = snprintf(buffer + offset, buffer_size - offset, "%02X%s", data[index], (index + 1U < len) ? " " : "");
		if (written < 0) {
			buffer[0] = '\0';
			return;
		}
		if ((size_t)written >= (buffer_size - offset)) {
			offset = buffer_size - 1U;
			break;
		}
		offset += (size_t)written;
	}
	buffer[offset] = '\0';
}

static esp_err_t install_gpio_isr_service_once(void)
{
	esp_err_t err = gpio_install_isr_service(0);
	if (err == ESP_ERR_INVALID_STATE) {
		return ESP_OK;
	}
	return err;
}

static void init_mcp2515_reset_pin(void)
{
	gpio_config_t reset_cfg = {
		.pin_bit_mask = (1ULL << HWCFG_MCP2515_RST_GPIO),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};

	ESP_ERROR_CHECK(gpio_config(&reset_cfg));
	ESP_ERROR_CHECK(gpio_set_level(HWCFG_MCP2515_RST_GPIO, 1));
}

static void init_mcp2515_bus(void)
{
	spi_bus_config_t bus_cfg = {
		.sclk_io_num = HWCFG_MCP2515_SCLK_GPIO,
		.mosi_io_num = HWCFG_MCP2515_MOSI_GPIO,
		.miso_io_num = HWCFG_MCP2515_MISO_GPIO,
		.quadwp_io_num = GPIO_NUM_NC,
		.quadhd_io_num = GPIO_NUM_NC,
	};

	init_mcp2515_reset_pin();
	ESP_ERROR_CHECK(spi_bus_initialize(HWCFG_MCP2515_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO));
	ESP_ERROR_CHECK(install_gpio_isr_service_once());
}

static void register_rx_callback(can_channel_ctx_t *channel)
{
	twai_event_callbacks_t callbacks = {
		.on_rx_done = can_rx_done_cb,
	};

	ESP_ERROR_CHECK(twai_node_register_event_callbacks(channel->node, &callbacks, channel));
}

static void init_internal_can_node(can_channel_ctx_t *channel)
{
	twai_onchip_node_config_t config = {0};

	config.io_cfg.tx = HWCFG_INTERNAL_CAN_TX_GPIO;
	config.io_cfg.rx = HWCFG_INTERNAL_CAN_RX_GPIO;
	config.io_cfg.quanta_clk_out = GPIO_NUM_NC;
	config.io_cfg.bus_off_indicator = GPIO_NUM_NC;
	config.bit_timing.bitrate = HWCFG_CAN_BITRATE;
	config.bit_timing.sp_permill = HWCFG_CAN_SAMPLE_POINT_PERMILL;
	config.fail_retry_cnt = -1;
	config.tx_queue_depth = APP_TX_QUEUE_DEPTH;
	config.intr_priority = 0;
	config.flags.enable_self_test = HWCFG_CAN_SELF_TEST_MODE;
	config.flags.enable_loopback = HWCFG_CAN_SELF_TEST_MODE;

	channel->name = channel_name(CAN_CHANNEL_INTERNAL);
	channel->channel_id = CAN_CHANNEL_INTERNAL;
	channel->rx_queue = s_app.rx_queue;

	ESP_ERROR_CHECK(twai_new_node_onchip(&config, &channel->node));
	register_rx_callback(channel);
	ESP_ERROR_CHECK(twai_node_enable(channel->node));
}

static void init_external_can_node(can_channel_ctx_t *channel)
{
	twai_mcp2515_node_config_t config = {
		.io_cfg = {
			.int_gpio = HWCFG_MCP2515_INT_GPIO,
			.cs_gpio = HWCFG_MCP2515_CS_GPIO,
		},
		.spi_clock_hz = HWCFG_MCP2515_SPI_CLOCK_HZ,
		.oscillator_hz = 0,
		.bit_timing = {
			.bitrate = HWCFG_CAN_BITRATE,
			.sp_permill = HWCFG_CAN_SAMPLE_POINT_PERMILL,
		},
		.fail_retry_cnt = -1,
		.timestamp_resolution_hz = APP_TIMESTAMP_RESOLUTION_HZ,
		.tx_queue_depth = APP_TX_QUEUE_DEPTH,
		.flags = {
			.enable_loopback = HWCFG_CAN_SELF_TEST_MODE,
		},
	};

	channel->name = channel_name(CAN_CHANNEL_EXTERNAL);
	channel->channel_id = CAN_CHANNEL_EXTERNAL;
	channel->rx_queue = s_app.rx_queue;

	init_mcp2515_bus();
	ESP_ERROR_CHECK(twai_new_node_mcp2515(HWCFG_MCP2515_SPI_HOST, &config, &channel->node));
	register_rx_callback(channel);
	ESP_ERROR_CHECK(twai_node_enable(channel->node));
}

static esp_err_t send_test_frame(const can_channel_ctx_t *channel, uint32_t frame_id, uint8_t channel_marker, uint32_t counter)
{
	uint8_t payload[8] = {
		channel_marker,
		(uint8_t)counter,
		(uint8_t)(counter >> 8),
		(uint8_t)(counter >> 16),
		(uint8_t)(counter >> 24),
		0xA5,
		0x5A,
		0xC3,
	};
	twai_frame_t frame = {
		.buffer = payload,
		.buffer_len = sizeof(payload),
	};
	esp_err_t err;

	frame.header.id = frame_id;

	err = twai_node_transmit(channel->node, &frame, APP_TX_TIMEOUT_MS);
	if (err != ESP_OK) {
		return err;
	}
	return twai_node_transmit_wait_all_done(channel->node, APP_TX_TIMEOUT_MS);
}

static void can_rx_logger_task(void *arg)
{
	QueueHandle_t rx_queue = (QueueHandle_t)arg;
	can_rx_event_t rx_event;
	char data_hex[(TWAI_FRAME_MAX_LEN * 3) + 1];

	while (true) {
		if (xQueueReceive(rx_queue, &rx_event, portMAX_DELAY) != pdTRUE) {
			continue;
		}

		format_data_hex(rx_event.data, rx_event.len, data_hex, sizeof(data_hex));
		ESP_LOGI(TAG,
				 "%s rx id=0x%03" PRIX32 " len=%u ts=%" PRIu64 " data=[%s]",
				 channel_name(rx_event.channel),
				 rx_event.id,
				 (unsigned int)rx_event.len,
				 rx_event.timestamp,
				 data_hex);
	}
}

static void can_test_task(void *arg)
{
	dual_can_app_t *app = (dual_can_app_t *)arg;
	uint32_t counter = 0;

	while (true) {
		esp_err_t internal_err = send_test_frame(&app->internal, APP_INTERNAL_TEST_ID, 0x11, counter);
		esp_err_t external_err = send_test_frame(&app->external, APP_EXTERNAL_TEST_ID, 0x22, counter);

		if (internal_err == ESP_OK) {
			ESP_LOGI(TAG, "%s tx round=%" PRIu32 " id=0x%03X", app->internal.name, counter, APP_INTERNAL_TEST_ID);
		} else {
			ESP_LOGE(TAG, "%s tx failed: %s", app->internal.name, esp_err_to_name(internal_err));
		}

		if (external_err == ESP_OK) {
			ESP_LOGI(TAG, "%s tx round=%" PRIu32 " id=0x%03X", app->external.name, counter, APP_EXTERNAL_TEST_ID);
		} else {
			ESP_LOGE(TAG, "%s tx failed: %s", app->external.name, esp_err_to_name(external_err));
		}

		counter++;
		vTaskDelay(pdMS_TO_TICKS(APP_TEST_FRAME_PERIOD_MS));
	}
}

static bool init_vbat_adc_calibration(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
	adc_cali_curve_fitting_config_t cali_cfg = {
		.unit_id = unit,
		.chan = channel,
		.atten = atten,
		.bitwidth = ADC_BITWIDTH_DEFAULT,
	};
	if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_vbat_cali_handle) == ESP_OK) {
		return true;
	}
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
	adc_cali_line_fitting_config_t cali_cfg = {
		.unit_id = unit,
		.atten = atten,
		.bitwidth = ADC_BITWIDTH_DEFAULT,
	};
	if (adc_cali_create_scheme_line_fitting(&cali_cfg, &s_vbat_cali_handle) == ESP_OK) {
		return true;
	}
#endif
	(void)unit;
	(void)channel;
	(void)atten;
	return false;
}

static void init_vbat_adc(void)
{
	adc_oneshot_unit_init_cfg_t unit_cfg = {
		.unit_id = ADC_UNIT_1,
		.ulp_mode = ADC_ULP_MODE_DISABLE,
	};
	adc_oneshot_chan_cfg_t chan_cfg = {
		.bitwidth = ADC_BITWIDTH_DEFAULT,
		.atten = APP_VBAT_ADC_ATTEN,
	};

	ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_vbat_adc_handle));
	ESP_ERROR_CHECK(adc_oneshot_config_channel(s_vbat_adc_handle, HWCFG_VBAT_ADC_CHANNEL, &chan_cfg));

	s_vbat_cali_enabled = init_vbat_adc_calibration(ADC_UNIT_1, HWCFG_VBAT_ADC_CHANNEL, APP_VBAT_ADC_ATTEN);
	if (s_vbat_cali_enabled) {
		ESP_LOGI(TAG, "VBAT ADC calibration enabled");
	} else {
		ESP_LOGW(TAG, "VBAT ADC calibration unavailable, using raw conversion");
	}
}

static void battery_monitor_task(void *arg)
{
	(void)arg;
	const float divider_ratio = (APP_VBAT_DIVIDER_R_TOP_OHM + APP_VBAT_DIVIDER_R_BOTTOM_OHM) / APP_VBAT_DIVIDER_R_BOTTOM_OHM;

	while (true) {
		int raw = 0;
		int pin_mv = 0;

		esp_err_t read_err = adc_oneshot_read(s_vbat_adc_handle, HWCFG_VBAT_ADC_CHANNEL, &raw);
		if (read_err != ESP_OK) {
			ESP_LOGE(TAG, "VBAT read failed: %s", esp_err_to_name(read_err));
			vTaskDelay(pdMS_TO_TICKS(APP_BATTERY_LOG_PERIOD_MS));
			continue;
		}

		if (s_vbat_cali_enabled) {
			esp_err_t cal_err = adc_cali_raw_to_voltage(s_vbat_cali_handle, raw, &pin_mv);
			if (cal_err != ESP_OK) {
				ESP_LOGE(TAG, "VBAT calibration conversion failed: %s", esp_err_to_name(cal_err));
				vTaskDelay(pdMS_TO_TICKS(APP_BATTERY_LOG_PERIOD_MS));
				continue;
			}
		} else {
			pin_mv = (raw * 3300) / 4095;
		}

		float battery_mv = pin_mv * divider_ratio;
		float battery_v = battery_mv / 1000.0f;
		ESP_LOGI(TAG, "VBAT=%.3f V (adc_raw=%d, adc_pin=%d mV)", battery_v, raw, pin_mv);

		vTaskDelay(pdMS_TO_TICKS(APP_BATTERY_LOG_PERIOD_MS));
	}
}

void app_main(void)
{
	s_app.rx_queue = xQueueCreate(APP_RX_QUEUE_DEPTH, sizeof(can_rx_event_t));
	ESP_ERROR_CHECK(s_app.rx_queue == NULL ? ESP_ERR_NO_MEM : ESP_OK);

	init_internal_can_node(&s_app.internal);
	init_external_can_node(&s_app.external);
	init_vbat_adc();

	ESP_LOGI(TAG,
			 "dual CAN self-test started, bitrate=%u, internal rx=%d tx=%d, mcp2515 int=%d cs=%d osc=%u",
			 HWCFG_CAN_BITRATE,
			 HWCFG_INTERNAL_CAN_RX_GPIO,
			 HWCFG_INTERNAL_CAN_TX_GPIO,
			 HWCFG_MCP2515_INT_GPIO,
			 HWCFG_MCP2515_CS_GPIO,
			 HWCFG_MCP2515_OSCILLATOR_HZ);

	BaseType_t rx_task_ok = xTaskCreate(can_rx_logger_task, "can_rx_logger", 4096, s_app.rx_queue, 5, NULL);
	BaseType_t tx_task_ok = xTaskCreate(can_test_task, "can_test_task", 4096, &s_app, 5, NULL);
	BaseType_t vbat_task_ok = xTaskCreate(battery_monitor_task, "battery_monitor", 3072, NULL, 4, NULL);

    gpio_reset_pin(HWCFG_YELLOW_LED_GPIO);
    gpio_set_direction(HWCFG_YELLOW_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(HWCFG_YELLOW_LED_GPIO, 0);

    gpio_reset_pin(HWCFG_GREEN_LED_GPIO);
    gpio_set_direction(HWCFG_GREEN_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(HWCFG_GREEN_LED_GPIO, 0);

    gpio_reset_pin(HWCFG_BLUE_LED_GPIO);
    gpio_set_direction(HWCFG_BLUE_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(HWCFG_BLUE_LED_GPIO, 0);

	ESP_ERROR_CHECK((rx_task_ok == pdPASS && tx_task_ok == pdPASS && vbat_task_ok == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM);
}
