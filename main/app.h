#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "esp_twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "mcp2518fd.h"

typedef enum {
	CAN_CHANNEL_INTERNAL = 0,
	CAN_CHANNEL_EXTERNAL,
} can_channel_id_t;

typedef struct {
	can_channel_id_t channel;
	uint32_t id;
	size_t len;
	uint64_t timestamp;
	bool extended;
	bool fd;
	bool brs;
	bool rtr;
	uint8_t data[MCP2518FD_MAX_DATA_LEN];
} can_rx_event_t;

typedef struct {
	const char *name;
	can_channel_id_t channel_id;
	twai_node_handle_t node;        /* internal channel (ESP32-S3 TWAI) */
	mcp2518fd_handle_t mcp;         /* external channel (MCP2518FD over SPI) */
	mcp2518fd_config_t mcp_cfg;     /* current external channel configuration */
	SemaphoreHandle_t lock;         /* guards mcp handle during reconfiguration */
	QueueHandle_t rx_queue;
} can_channel_ctx_t;

typedef struct {
	QueueHandle_t rx_queue;
	can_channel_ctx_t internal;
	can_channel_ctx_t external;
	volatile bool test_traffic;     /* periodic 0x611/0x612/0x7DF test frames */
	volatile bool rx_print;         /* print received frames to the console */
	uint32_t internal_bitrate;
} dual_can_app_t;

extern dual_can_app_t s_app;

const char *app_channel_name(can_channel_id_t channel_id);
const char *app_can_mode_to_string(mcp2518fd_mode_t mode);
bool app_parse_can_mode(const char *text, mcp2518fd_mode_t *out_mode);

/* Send helpers (payload up to 64 bytes on external, 8 on internal) */
esp_err_t app_send_internal(uint32_t id, bool extended, const uint8_t *data, size_t len);
esp_err_t app_send_external(uint32_t id, bool extended, bool fd, bool brs, const uint8_t *data, size_t len);

/* Re-create the MCP2518FD node with a new configuration (bitrates / mode / fd / brs) */
esp_err_t app_external_reconfigure(const mcp2518fd_config_t *new_cfg);
/* Re-create the on-chip TWAI node with a new bitrate */
esp_err_t app_internal_reconfigure(uint32_t bitrate);

/* Status printing */
void app_print_status(void);

/* Battery voltage in volts (NAN on error) */
float app_read_vbat(void);

/* Power down everything and sleep. seconds == 0 -> deep sleep until reset. */
void app_enter_sleep(uint32_t seconds, bool light_sleep);

void app_console_start(void);
