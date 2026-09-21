#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "app.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "hw_config.h"
#include "mcp2518fd.h"

#define APP_RX_QUEUE_DEPTH               32U
#define APP_TEST_FRAME_PERIOD_MS         1000U
#define APP_TX_QUEUE_DEPTH               8U
#define APP_TX_TIMEOUT_MS                1000
#define APP_INTERNAL_TEST_ID             0x611U
#define APP_EXTERNAL_TEST_ID             0x612U
#define APP_OBD_REQUEST_ID               0x7DFU   /* OBD-II functional request: PID 0x0C (engine RPM) */
#define APP_OBD_PID                      0x0CU
#define APP_EXT_RX_POLL_MS               50U
#define APP_LOCK_TIMEOUT_MS              2000U

#define APP_VBAT_DIVIDER_R_TOP_OHM       62000.0f
#define APP_VBAT_DIVIDER_R_BOTTOM_OHM    6200.0f
#define APP_VBAT_ADC_ATTEN               ADC_ATTEN_DB_6

static const char *TAG = "dual_can_test";

dual_can_app_t s_app;
static adc_oneshot_unit_handle_t s_vbat_adc_handle;
static adc_cali_handle_t s_vbat_cali_handle;
static bool s_vbat_cali_enabled;

const char *app_channel_name(can_channel_id_t channel_id)
{
	return (channel_id == CAN_CHANNEL_INTERNAL) ? "internal" : "external";
}

const char *app_can_mode_to_string(mcp2518fd_mode_t mode)
{
	switch (mode) {
	case MCP2518FD_MODE_NORMAL_FD: return "normal-fd";
	case MCP2518FD_MODE_SLEEP: return "sleep";
	case MCP2518FD_MODE_INTERNAL_LOOPBACK: return "internal-loopback";
	case MCP2518FD_MODE_LISTEN_ONLY: return "listen-only";
	case MCP2518FD_MODE_CONFIGURATION: return "configuration";
	case MCP2518FD_MODE_EXTERNAL_LOOPBACK: return "external-loopback";
	case MCP2518FD_MODE_NORMAL_CLASSIC: return "normal-classic";
	case MCP2518FD_MODE_RESTRICTED: return "restricted";
	default: return "unknown";
	}
}

bool app_parse_can_mode(const char *text, mcp2518fd_mode_t *out_mode)
{
	if (strcasecmp(text, "fd") == 0 || strcasecmp(text, "normal-fd") == 0) {
		*out_mode = MCP2518FD_MODE_NORMAL_FD;
	} else if (strcasecmp(text, "classic") == 0 || strcasecmp(text, "normal-classic") == 0) {
		*out_mode = MCP2518FD_MODE_NORMAL_CLASSIC;
	} else if (strcasecmp(text, "listen") == 0 || strcasecmp(text, "listen-only") == 0) {
		*out_mode = MCP2518FD_MODE_LISTEN_ONLY;
	} else if (strcasecmp(text, "iloop") == 0 || strcasecmp(text, "internal-loopback") == 0) {
		*out_mode = MCP2518FD_MODE_INTERNAL_LOOPBACK;
	} else if (strcasecmp(text, "eloop") == 0 || strcasecmp(text, "external-loopback") == 0) {
		*out_mode = MCP2518FD_MODE_EXTERNAL_LOOPBACK;
	} else {
		return false;
	}
	return true;
}

/* ── GPIO helpers ─────────────────────────────────────────── */

static void init_output_low(gpio_num_t gpio)
{
	gpio_config_t cfg = {
		.pin_bit_mask = (1ULL << gpio),
		.mode = GPIO_MODE_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};

	ESP_ERROR_CHECK(gpio_config(&cfg));
	ESP_ERROR_CHECK(gpio_set_level(gpio, 0));
}

static void release_to_input(gpio_num_t gpio)
{
	/* Let the on-board pull-up define the level (transceiver standby) */
	gpio_config_t cfg = {
		.pin_bit_mask = (1ULL << gpio),
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};
	gpio_config(&cfg);
}

static void leds_set(bool on)
{
	const gpio_num_t leds[] = { HWCFG_YELLOW_LED_GPIO, HWCFG_GREEN_LED_GPIO, HWCFG_BLUE_LED_GPIO };
	for (size_t i = 0; i < sizeof(leds) / sizeof(leds[0]); i++) {
		gpio_reset_pin(leds[i]);
		gpio_set_direction(leds[i], GPIO_MODE_OUTPUT);
		gpio_set_level(leds[i], on ? 0 : 1);   /* active low */
	}
}

/* ── Internal channel: on-chip TWAI ───────────────────────── */

static bool can_rx_done_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
	(void)edata;
	can_channel_ctx_t *channel = (can_channel_ctx_t *)user_ctx;
	can_rx_event_t rx_event = {
		.channel = channel->channel_id,
	};
	twai_frame_t rx_frame = {
		.buffer = rx_event.data,
		.buffer_len = TWAI_FRAME_MAX_LEN,
	};

	if (twai_node_receive_from_isr(handle, &rx_frame) != ESP_OK) {
		return false;
	}

	rx_event.id = rx_frame.header.id;
	rx_event.len = rx_frame.buffer_len;
	rx_event.timestamp = rx_frame.header.timestamp;
	rx_event.extended = rx_frame.header.ide;
	rx_event.rtr = rx_frame.header.rtr;

	BaseType_t task_woken = pdFALSE;
	xQueueSendFromISR(channel->rx_queue, &rx_event, &task_woken);
	return (task_woken == pdTRUE);
}

static esp_err_t create_internal_node(can_channel_ctx_t *channel, uint32_t bitrate)
{
	twai_onchip_node_config_t config = {0};

	config.io_cfg.tx = HWCFG_INTERNAL_CAN_TX_GPIO;
	config.io_cfg.rx = HWCFG_INTERNAL_CAN_RX_GPIO;
	config.io_cfg.quanta_clk_out = GPIO_NUM_NC;
	config.io_cfg.bus_off_indicator = GPIO_NUM_NC;
	config.bit_timing.bitrate = bitrate;
	config.bit_timing.sp_permill = HWCFG_CAN_SAMPLE_POINT_PERMILL;
	config.fail_retry_cnt = -1;
	config.tx_queue_depth = APP_TX_QUEUE_DEPTH;
	config.intr_priority = 0;
	config.flags.enable_self_test = HWCFG_CAN_SELF_TEST_MODE;
	config.flags.enable_loopback = HWCFG_CAN_SELF_TEST_MODE;

	esp_err_t err = twai_new_node_onchip(&config, &channel->node);
	if (err != ESP_OK) {
		return err;
	}
	twai_event_callbacks_t callbacks = {
		.on_rx_done = can_rx_done_cb,
	};
	err = twai_node_register_event_callbacks(channel->node, &callbacks, channel);
	if (err == ESP_OK) {
		err = twai_node_enable(channel->node);
	}
	return err;
}

static void init_internal_can_node(can_channel_ctx_t *channel)
{
	/* SN65HVD233 RS pin is pulled up (standby) on the board; drive it low for high-speed mode. */
	init_output_low(HWCFG_INTERNAL_CAN_STDBY_GPIO);

	channel->name = app_channel_name(CAN_CHANNEL_INTERNAL);
	channel->channel_id = CAN_CHANNEL_INTERNAL;
	channel->rx_queue = s_app.rx_queue;
	channel->lock = xSemaphoreCreateMutex();
	s_app.internal_bitrate = HWCFG_CAN_BITRATE;

	ESP_ERROR_CHECK(create_internal_node(channel, s_app.internal_bitrate));
}

esp_err_t app_internal_reconfigure(uint32_t bitrate)
{
	can_channel_ctx_t *channel = &s_app.internal;
	if (xSemaphoreTake(channel->lock, pdMS_TO_TICKS(APP_LOCK_TIMEOUT_MS)) != pdTRUE) {
		return ESP_ERR_TIMEOUT;
	}
	esp_err_t err = ESP_OK;
	if (channel->node) {
		twai_node_disable(channel->node);
		twai_node_delete(channel->node);
		channel->node = NULL;
	}
	err = create_internal_node(channel, bitrate);
	if (err == ESP_OK) {
		s_app.internal_bitrate = bitrate;
	}
	xSemaphoreGive(channel->lock);
	return err;
}

/* ── External channel: MCP2518FD over SPI ─────────────────── */

/* The RX task must not hold the channel mutex while blocked in receive (it would
 * starve lower-priority senders), so reconfiguration parks it via hold/idle flags. */
static volatile bool s_ext_rx_hold;
static volatile bool s_ext_rx_idle;

static void external_rx_park(void)
{
	s_ext_rx_hold = true;
	for (int i = 0; i < 50 && !s_ext_rx_idle; i++) {
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}

static void external_rx_resume(void)
{
	s_ext_rx_hold = false;
}

static void external_can_rx_task(void *arg)
{
	can_channel_ctx_t *channel = (can_channel_ctx_t *)arg;
	mcp2518fd_frame_t frame;

	while (true) {
		if (s_ext_rx_hold || channel->mcp == NULL) {
			s_ext_rx_idle = true;
			vTaskDelay(pdMS_TO_TICKS(20));
			continue;
		}
		s_ext_rx_idle = false;
		esp_err_t err = mcp2518fd_receive(channel->mcp, &frame, pdMS_TO_TICKS(APP_EXT_RX_POLL_MS));
		if (err != ESP_OK) {
			continue;
		}

		can_rx_event_t rx_event = {
			.channel = channel->channel_id,
			.id = frame.id,
			.len = frame.data_length,
			.timestamp = frame.timestamp,
			.extended = frame.extended_id,
			.fd = frame.fd_frame,
			.brs = frame.bit_rate_switch,
			.rtr = frame.remote_frame,
		};
		if (rx_event.len > sizeof(rx_event.data)) {
			rx_event.len = sizeof(rx_event.data);
		}
		memcpy(rx_event.data, frame.data, rx_event.len);

		if (xQueueSend(channel->rx_queue, &rx_event, pdMS_TO_TICKS(APP_EXT_RX_POLL_MS)) != pdTRUE) {
			ESP_LOGW(TAG, "%s rx queue full, dropping id=0x%03" PRIX32, channel->name, rx_event.id);
		}
	}
}

static mcp2518fd_config_t default_external_config(void)
{
	mcp2518fd_config_t config = MCP2518FD_CONFIG_DEFAULT();

	config.spi_host = HWCFG_MCP2518FD_SPI_HOST;
	config.pin_sclk = HWCFG_MCP2518FD_SCLK_GPIO;
	config.pin_mosi = HWCFG_MCP2518FD_MOSI_GPIO;
	config.pin_miso = HWCFG_MCP2518FD_MISO_GPIO;
	config.pin_cs = HWCFG_MCP2518FD_CS_GPIO;
	config.pin_int = HWCFG_MCP2518FD_INT_GPIO;
	config.pin_standby = HWCFG_MCP2518FD_STDBY_GPIO;   /* TCAN3413 STB, driven low by the driver */
	config.spi_clock_hz = HWCFG_MCP2518FD_SPI_CLOCK_HZ;
	config.oscillator_hz = HWCFG_MCP2518FD_OSCILLATOR_HZ;
	config.nominal_bitrate = HWCFG_CAN_BITRATE;
	config.data_bitrate = HWCFG_MCP2518FD_DATA_BITRATE;
	config.tx_queue_len = APP_TX_QUEUE_DEPTH;
	config.rx_queue_len = APP_RX_QUEUE_DEPTH;
	config.initialize_spi_bus = true;
#if HWCFG_CAN_SELF_TEST_MODE || HWCFG_MCP2518FD_LOOPBACK
	config.mode = MCP2518FD_MODE_INTERNAL_LOOPBACK;
	config.enable_can_fd = HWCFG_MCP2518FD_ENABLE_FD;
	config.enable_brs = HWCFG_MCP2518FD_ENABLE_FD;
#elif HWCFG_MCP2518FD_ENABLE_FD
	config.mode = MCP2518FD_MODE_NORMAL_FD;
	config.enable_can_fd = true;
	config.enable_brs = true;
#else
	config.mode = MCP2518FD_MODE_NORMAL_CLASSIC;
	config.enable_can_fd = false;
	config.enable_brs = false;
#endif
	return config;
}

static esp_err_t create_external_node(can_channel_ctx_t *channel, const mcp2518fd_config_t *cfg)
{
	esp_err_t err = mcp2518fd_new(cfg, &channel->mcp);
	if (err != ESP_OK) {
		channel->mcp = NULL;
		return err;
	}
	err = mcp2518fd_start(channel->mcp);
	if (err != ESP_OK) {
		mcp2518fd_delete(channel->mcp);
		channel->mcp = NULL;
		return err;
	}
	channel->mcp_cfg = *cfg;
	return ESP_OK;
}

static void init_external_can_node(can_channel_ctx_t *channel)
{
	mcp2518fd_config_t config = default_external_config();

	channel->name = app_channel_name(CAN_CHANNEL_EXTERNAL);
	channel->channel_id = CAN_CHANNEL_EXTERNAL;
	channel->rx_queue = s_app.rx_queue;
	channel->lock = xSemaphoreCreateMutex();

	ESP_ERROR_CHECK(create_external_node(channel, &config));

	mcp2518fd_diagnostics_t diag = {0};
	if (mcp2518fd_get_diagnostics(channel->mcp, &diag) == ESP_OK) {
		ESP_LOGI(TAG, "mcp2518fd devid=0x%08" PRIX32 " osc=0x%08" PRIX32 " c1con=0x%08" PRIX32 " reqop=%u opmod=%u",
				 diag.devid, diag.osc, diag.c1con, diag.requested_mode, diag.operating_mode);
	}

	BaseType_t ok = xTaskCreate(external_can_rx_task, "ext_can_rx", 4096, channel, 6, NULL);
	ESP_ERROR_CHECK(ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM);
}

esp_err_t app_external_reconfigure(const mcp2518fd_config_t *new_cfg)
{
	can_channel_ctx_t *channel = &s_app.external;
	if (xSemaphoreTake(channel->lock, pdMS_TO_TICKS(APP_LOCK_TIMEOUT_MS)) != pdTRUE) {
		return ESP_ERR_TIMEOUT;
	}
	external_rx_park();
	if (channel->mcp) {
		mcp2518fd_handle_t old_handle = channel->mcp;
		channel->mcp = NULL;
		mcp2518fd_stop(old_handle);
		mcp2518fd_delete(old_handle);   /* frees the SPI bus too (owns_spi_bus) */
	}
	esp_err_t err = create_external_node(channel, new_cfg);
	if (err != ESP_OK) {
		/* fall back to the previous configuration so the channel stays usable */
		mcp2518fd_config_t old = channel->mcp_cfg;
		esp_err_t err2 = create_external_node(channel, &old);
		ESP_LOGW(TAG, "external reconfigure failed (%s), restored previous config: %s",
				 esp_err_to_name(err), esp_err_to_name(err2));
	}
	external_rx_resume();
	xSemaphoreGive(channel->lock);
	return err;
}

/* ── Sending ──────────────────────────────────────────────── */

esp_err_t app_send_internal(uint32_t id, bool extended, const uint8_t *data, size_t len)
{
	can_channel_ctx_t *channel = &s_app.internal;
	if (len > TWAI_FRAME_MAX_LEN) {
		return ESP_ERR_INVALID_ARG;
	}
	if (xSemaphoreTake(channel->lock, pdMS_TO_TICKS(APP_LOCK_TIMEOUT_MS)) != pdTRUE) {
		return ESP_ERR_TIMEOUT;
	}
	esp_err_t err = ESP_ERR_INVALID_STATE;
	if (channel->node) {
		twai_frame_t frame = {
			.buffer = (uint8_t *)data,
			.buffer_len = len,
		};
		frame.header.id = id;
		frame.header.ide = extended;
		err = twai_node_transmit(channel->node, &frame, APP_TX_TIMEOUT_MS);
		if (err == ESP_OK) {
			err = twai_node_transmit_wait_all_done(channel->node, APP_TX_TIMEOUT_MS);
		}
	}
	xSemaphoreGive(channel->lock);
	return err;
}

esp_err_t app_send_external(uint32_t id, bool extended, bool fd, bool brs, const uint8_t *data, size_t len)
{
	can_channel_ctx_t *channel = &s_app.external;
	if (len > MCP2518FD_MAX_DATA_LEN) {
		return ESP_ERR_INVALID_ARG;
	}
	size_t data_len = len;
	if (fd && len > 8) {
		/* pad to the next valid CAN FD DLC length */
		static const uint8_t fd_lengths[] = { 12, 16, 20, 24, 32, 48, 64 };
		for (size_t i = 0; i < sizeof(fd_lengths); i++) {
			if (len <= fd_lengths[i]) {
				len = fd_lengths[i];
				break;
			}
		}
	}
	mcp2518fd_frame_t frame = {
		.id = id,
		.data_length = (uint8_t)len,
		.extended_id = extended,
		.fd_frame = fd,
		.bit_rate_switch = brs,
		.remote_frame = false,
	};
	memcpy(frame.data, data, data_len);

	if (xSemaphoreTake(channel->lock, pdMS_TO_TICKS(APP_LOCK_TIMEOUT_MS)) != pdTRUE) {
		return ESP_ERR_TIMEOUT;
	}
	esp_err_t err = ESP_ERR_INVALID_STATE;
	if (channel->mcp) {
		err = mcp2518fd_transmit(channel->mcp, &frame, pdMS_TO_TICKS(APP_TX_TIMEOUT_MS));
	}
	xSemaphoreGive(channel->lock);
	return err;
}

/* ── Test traffic ─────────────────────────────────────────── */

static void fill_test_payload(uint8_t *payload, size_t len, uint8_t channel_marker, uint32_t counter)
{
	memset(payload, 0, len);
	payload[0] = channel_marker;
	payload[1] = (uint8_t)counter;
	payload[2] = (uint8_t)(counter >> 8);
	payload[3] = (uint8_t)(counter >> 16);
	payload[4] = (uint8_t)(counter >> 24);
	payload[5] = 0xA5;
	payload[6] = 0x5A;
	payload[7] = 0xC3;
	for (size_t i = 8; i < len; i++) {
		payload[i] = (uint8_t)i;
	}
}

static void format_data_hex(const uint8_t *data, size_t len, char *buffer, size_t buffer_size)
{
	size_t offset = 0;
	buffer[0] = '\0';
	for (size_t index = 0; index < len && offset + 3 < buffer_size; index++) {
		int written = snprintf(buffer + offset, buffer_size - offset, "%02X%s", data[index], (index + 1U < len) ? " " : "");
		if (written < 0 || (size_t)written >= (buffer_size - offset)) {
			break;
		}
		offset += (size_t)written;
	}
	buffer[offset] = '\0';
}

static void can_rx_logger_task(void *arg)
{
	QueueHandle_t rx_queue = (QueueHandle_t)arg;
	can_rx_event_t rx_event;
	char data_hex[(MCP2518FD_MAX_DATA_LEN * 3) + 1];

	while (true) {
		if (xQueueReceive(rx_queue, &rx_event, portMAX_DELAY) != pdTRUE) {
			continue;
		}
		if (!s_app.rx_print) {
			continue;
		}
		format_data_hex(rx_event.data, rx_event.len, data_hex, sizeof(data_hex));
		ESP_LOGI(TAG,
				 "%s rx id=0x%0*" PRIX32 "%s%s%s len=%u ts=%" PRIu64 " data=[%s]",
				 app_channel_name(rx_event.channel),
				 rx_event.extended ? 8 : 3,
				 rx_event.id,
				 rx_event.extended ? " ext" : "",
				 rx_event.fd ? " fd" : "",
				 rx_event.brs ? " brs" : "",
				 (unsigned int)rx_event.len,
				 rx_event.timestamp,
				 data_hex);
	}
}

static void log_tx_result(const char *channel, const char *what, uint32_t id, esp_err_t err)
{
	if (err == ESP_OK) {
		ESP_LOGI(TAG, "%s tx %s id=0x%03" PRIX32 " ok", channel, what, id);
	} else {
		ESP_LOGE(TAG, "%s tx %s id=0x%03" PRIX32 " failed: %s", channel, what, id, esp_err_to_name(err));
	}
}

static void can_test_task(void *arg)
{
	(void)arg;
	uint32_t counter = 0;
	uint8_t obd_request[8] = { 0x02, 0x01, APP_OBD_PID, 0, 0, 0, 0, 0 };
	uint8_t payload[MCP2518FD_MAX_DATA_LEN];

	while (true) {
		if (!s_app.test_traffic) {
			vTaskDelay(pdMS_TO_TICKS(100));
			continue;
		}
		ESP_LOGI(TAG, "---- round %" PRIu32 " ----", counter);

		fill_test_payload(payload, 8, 0x11, counter);
		log_tx_result("internal", "test", APP_INTERNAL_TEST_ID, app_send_internal(APP_INTERNAL_TEST_ID, false, payload, 8));

		bool fd = s_app.external.mcp_cfg.enable_can_fd;
		bool brs = fd && s_app.external.mcp_cfg.enable_brs;
		size_t len = fd ? MCP2518FD_MAX_DATA_LEN : 8;
		fill_test_payload(payload, len, 0x22, counter);
		log_tx_result("external", fd ? "test(fd)" : "test", APP_EXTERNAL_TEST_ID,
					  app_send_external(APP_EXTERNAL_TEST_ID, false, fd, brs, payload, len));

		/* OBD-II request on both channels: whichever bus has the ECU (simulator) answers with 0x7E8 */
		log_tx_result("internal", "obd", APP_OBD_REQUEST_ID, app_send_internal(APP_OBD_REQUEST_ID, false, obd_request, 8));
		log_tx_result("external", "obd", APP_OBD_REQUEST_ID, app_send_external(APP_OBD_REQUEST_ID, false, false, false, obd_request, 8));

		if ((counter % 5U) == 4U) {
			app_print_status();
		}

		counter++;
		vTaskDelay(pdMS_TO_TICKS(APP_TEST_FRAME_PERIOD_MS));
	}
}

/* ── Status ───────────────────────────────────────────────── */

void app_print_status(void)
{
	twai_node_status_t status = {0};
	if (s_app.internal.node && twai_node_get_info(s_app.internal.node, &status, NULL) == ESP_OK) {
		printf("internal: twai bitrate=%" PRIu32 " state=%d tec=%u rec=%u\n",
			   s_app.internal_bitrate, status.state, status.tx_error_count, status.rx_error_count);
	} else {
		printf("internal: not running\n");
	}

	const mcp2518fd_config_t *c = &s_app.external.mcp_cfg;
	printf("external: mcp2518fd mode=%s fd=%d brs=%d nominal=%" PRIu32 " data=%" PRIu32 " osc=%" PRIu32 " spi=%" PRIu32 "\n",
		   app_can_mode_to_string(c->mode), c->enable_can_fd, c->enable_brs, c->nominal_bitrate, c->data_bitrate,
		   c->oscillator_hz, c->spi_clock_hz);
	mcp2518fd_diagnostics_t diag = {0};
	if (s_app.external.mcp && mcp2518fd_get_diagnostics(s_app.external.mcp, &diag) == ESP_OK) {
		printf("          opmod=%u tec=%u rec=%u trec=0x%08" PRIX32 " txqsta=0x%08" PRIX32 " bdiag0=0x%08" PRIX32 " bdiag1=0x%08" PRIX32 "\n",
			   diag.operating_mode, diag.tx_error_count, diag.rx_error_count, diag.trec, diag.txqsta, diag.bdiag0, diag.bdiag1);
	} else {
		printf("          not running\n");
	}
	printf("test traffic=%s rxprint=%s VBAT=%.3f V\n", s_app.test_traffic ? "on" : "off", s_app.rx_print ? "on" : "off", app_read_vbat());
}

/* ── Battery monitor ──────────────────────────────────────── */

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
	if (!s_vbat_cali_enabled) {
		ESP_LOGW(TAG, "VBAT ADC calibration unavailable, using raw conversion");
	}
}

float app_read_vbat(void)
{
	const float divider_ratio = (APP_VBAT_DIVIDER_R_TOP_OHM + APP_VBAT_DIVIDER_R_BOTTOM_OHM) / APP_VBAT_DIVIDER_R_BOTTOM_OHM;
	int raw = 0;
	int pin_mv = 0;

	if (adc_oneshot_read(s_vbat_adc_handle, HWCFG_VBAT_ADC_CHANNEL, &raw) != ESP_OK) {
		return NAN;
	}
	if (s_vbat_cali_enabled) {
		if (adc_cali_raw_to_voltage(s_vbat_cali_handle, raw, &pin_mv) != ESP_OK) {
			return NAN;
		}
	} else {
		pin_mv = (raw * 3300) / 4095;
	}
	return (pin_mv * divider_ratio) / 1000.0f;
}

/* ── Sleep ────────────────────────────────────────────────── */

void app_enter_sleep(uint32_t seconds, bool light_sleep)
{
	s_app.test_traffic = false;
	printf("entering %s sleep%s...\n", light_sleep ? "light" : "deep", seconds ? "" : " (until reset)");
	if (seconds) {
		printf("wake in %" PRIu32 " s\n", seconds);
	}
	fflush(stdout);
	vTaskDelay(pdMS_TO_TICKS(100));

	/* External: MCP2518FD to low-power sleep, TCAN3413 to standby (release STB to its pull-up) */
	xSemaphoreTake(s_app.external.lock, portMAX_DELAY);
	external_rx_park();
	if (s_app.external.mcp) {
		esp_err_t err = mcp2518fd_enter_sleep(s_app.external.mcp, true);
		if (err != ESP_OK) {
			ESP_LOGW(TAG, "mcp2518fd sleep: %s", esp_err_to_name(err));
		}
	}
	release_to_input(HWCFG_MCP2518FD_STDBY_GPIO);

	/* Internal: stop TWAI, SN65HVD233 RS to its pull-up (standby) */
	xSemaphoreTake(s_app.internal.lock, portMAX_DELAY);
	if (s_app.internal.node) {
		twai_node_disable(s_app.internal.node);
	}
	release_to_input(HWCFG_INTERNAL_CAN_STDBY_GPIO);

	leds_set(false);

	/* Keep the transceiver standby pins at their pull-up level even while the ESP32 sleeps */
	gpio_hold_en(HWCFG_MCP2518FD_STDBY_GPIO);
	gpio_hold_en(HWCFG_INTERNAL_CAN_STDBY_GPIO);
	gpio_deep_sleep_hold_en();

	if (seconds) {
		esp_sleep_enable_timer_wakeup((uint64_t)seconds * 1000000ULL);
	}

	if (light_sleep) {
		esp_light_sleep_start();
		/* --- woke up --- */
		gpio_deep_sleep_hold_dis();
		gpio_hold_dis(HWCFG_MCP2518FD_STDBY_GPIO);
		gpio_hold_dis(HWCFG_INTERNAL_CAN_STDBY_GPIO);
		init_output_low(HWCFG_INTERNAL_CAN_STDBY_GPIO);
		if (s_app.internal.node) {
			twai_node_enable(s_app.internal.node);
		}
		xSemaphoreGive(s_app.internal.lock);
		if (s_app.external.mcp) {
			/* driver's start() performs a full reset + re-init and re-drives STB low */
			esp_err_t err = mcp2518fd_start(s_app.external.mcp);
			if (err != ESP_OK) {
				ESP_LOGW(TAG, "mcp2518fd restart after sleep: %s", esp_err_to_name(err));
			}
		}
		external_rx_resume();
		xSemaphoreGive(s_app.external.lock);
		leds_set(true);
		printf("woke from light sleep\n");
		return;
	}

	esp_deep_sleep_start();
}

/* ── Entry ────────────────────────────────────────────────── */

void app_main(void)
{
	esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
	if (cause != ESP_SLEEP_WAKEUP_UNDEFINED) {
		gpio_deep_sleep_hold_dis();
		gpio_hold_dis(HWCFG_MCP2518FD_STDBY_GPIO);
		gpio_hold_dis(HWCFG_INTERNAL_CAN_STDBY_GPIO);
		ESP_LOGI(TAG, "woke from deep sleep (cause=%d)", cause);
	}

	s_app.rx_queue = xQueueCreate(APP_RX_QUEUE_DEPTH, sizeof(can_rx_event_t));
	ESP_ERROR_CHECK(s_app.rx_queue == NULL ? ESP_ERR_NO_MEM : ESP_OK);
	s_app.test_traffic = true;
	s_app.rx_print = true;

	leds_set(true);
	init_internal_can_node(&s_app.internal);
	init_external_can_node(&s_app.external);
	init_vbat_adc();

	ESP_LOGI(TAG,
			 "dual CAN test started, bitrate=%u, internal rx=%d tx=%d stdby=%d, mcp2518fd int=%d cs=%d stdby=%d osc=%u fd=%d loopback=%d",
			 HWCFG_CAN_BITRATE,
			 HWCFG_INTERNAL_CAN_RX_GPIO,
			 HWCFG_INTERNAL_CAN_TX_GPIO,
			 HWCFG_INTERNAL_CAN_STDBY_GPIO,
			 HWCFG_MCP2518FD_INT_GPIO,
			 HWCFG_MCP2518FD_CS_GPIO,
			 HWCFG_MCP2518FD_STDBY_GPIO,
			 HWCFG_MCP2518FD_OSCILLATOR_HZ,
			 HWCFG_MCP2518FD_ENABLE_FD,
			 HWCFG_MCP2518FD_LOOPBACK);

	BaseType_t rx_task_ok = xTaskCreate(can_rx_logger_task, "can_rx_logger", 4096, s_app.rx_queue, 5, NULL);
	BaseType_t tx_task_ok = xTaskCreate(can_test_task, "can_test_task", 4096, NULL, 5, NULL);
	ESP_ERROR_CHECK((rx_task_ok == pdPASS && tx_task_ok == pdPASS) ? ESP_OK : ESP_ERR_NO_MEM);

	app_console_start();
}
