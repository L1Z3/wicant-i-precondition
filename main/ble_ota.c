/*
 * Firmware update over BLE.
 *
 * The web page's update (config_server.c) needs a phone or laptop on the
 * WiCAN's Wi-Fi. That isn't always possible: with the car on, a head unit
 * running wireless CarPlay or Android Auto keeps the phone on its own Wi-Fi.
 * This takes the image over the BLE link instead, on the same command
 * characteristic as the ELM327 commands:
 *
 *   ATXOB          -> XOB:1          the command exists (a client's probe)
 *   ATXOB<size>    -> XOB:READY      or XOB:ERR,<reason>
 *   <size> bytes   written to the command characteristic, with response,
 *                  so the link paces itself
 *   ATXOBEND       -> XOB:DONE       image checked and set to boot; reboots
 *                                    500 ms later
 *                  -> XOB:ERR,IMAGE    the image failed its check
 *                  -> XOB:ERR,VARIANT  the image is for another board
 *
 * A write that fails mid-transfer sends $XOB:ERR,<reason> at once. A dropped
 * link abandons the transfer. The image goes through esp_ota_* as the web
 * page's upload does, so esp_ota_end checks it before it can boot, and an
 * image for another board variant is refused the same way. Nothing here is
 * heap until a transfer starts. tools/ble_ota/ble_ota.py is a desktop client.
 */
#include "ble_ota.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "types.h"

static const char *TAG = "ble_ota";

static QueueHandle_t *s_tx_queue;
static esp_ota_handle_t s_handle;
static const esp_partition_t *s_partition;
static uint32_t s_expected;
static uint32_t s_received;
// Image bytes expected. Set by ble_ota_command (the ELM327 task) once the
// handle is open, cleared by the BTC task on the last byte, a failed write or
// a drop.
static volatile bool s_active;
// Every byte is written and the handle is open, waiting for ATXOBEND.
static volatile bool s_complete;
static esp_timer_handle_t s_reboot_timer;

void ble_ota_set_tx_queue(QueueHandle_t *queue)
{
	s_tx_queue = queue;
}

bool ble_ota_active(void)
{
	return s_active;
}

static void ble_ota_notify(const char *line)
{
	static xdev_buffer buffer;
	if (s_tx_queue == NULL || *s_tx_queue == NULL) return;
	int len = snprintf((char *)buffer.ucElement, sizeof(buffer.ucElement), "$%s\r", line);
	if (len <= 0) return;
	if (len >= (int)sizeof(buffer.ucElement)) len = sizeof(buffer.ucElement) - 1;
	buffer.usLen = len;
	buffer.dev_channel = DEV_BLE;
	xQueueSend(*s_tx_queue, &buffer, pdMS_TO_TICKS(100));
}

static void reboot_cb(void *arg)
{
	(void)arg;
	esp_restart();
}

static void schedule_reboot(void)
{
	const esp_timer_create_args_t args = { .callback = reboot_cb, .name = "ble_ota_reboot" };
	if (s_reboot_timer == NULL && esp_timer_create(&args, &s_reboot_timer) != ESP_OK)
	{
		esp_restart();
	}
	// Long enough for the reply to go out first.
	esp_timer_start_once(s_reboot_timer, 500 * 1000);
}

/* The board variant in an image's name, the field after "wican-fw_", as
 * config_server.c's upload handler reads it ("custom" is the old name for
 * "proto"). False for any other name. */
static bool app_variant(const esp_app_desc_t *app, char *out, size_t out_len)
{
	static const char prefix[] = "wican-fw_";
	const size_t prefix_len = sizeof(prefix) - 1;
	const char *name = app->project_name;
	size_t name_len = strnlen(name, sizeof(app->project_name));

	if (name_len <= prefix_len || strncmp(name, prefix, prefix_len) != 0)
	{
		return false;
	}
	const char *field = name + prefix_len;
	const char *end = memchr(field, '_', name_len - prefix_len);
	size_t len = end ? (size_t)(end - field) : name_len - prefix_len;

	if (len == 0 || len >= out_len)
	{
		return false;
	}
	memcpy(out, field, len);
	out[len] = '\0';
	if (strcmp(out, "custom") == 0)
	{
		strlcpy(out, "proto", out_len);
	}
	return true;
}

static const char *ble_ota_begin(const char *size_str)
{
	char *end = NULL;
	unsigned long size = strtoul(size_str, &end, 10);
	if (end == size_str || *end != 0) return NULL;

	// A finished transfer nobody ended: let it go.
	if (s_complete)
	{
		esp_ota_abort(s_handle);
		s_complete = false;
	}
	s_partition = esp_ota_get_next_update_partition(NULL);
	if (s_partition == NULL || size < 1024 || size > s_partition->size)
	{
		return "XOB:ERR,SIZE";
	}
	// Sequential writes erase each sector as it's reached, instead of the
	// whole partition up front, which would hold up this task for seconds.
	esp_err_t err = esp_ota_begin(s_partition, OTA_WITH_SEQUENTIAL_WRITES, &s_handle);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "esp_ota_begin failed (%s)", esp_err_to_name(err));
		return "XOB:ERR,BEGIN";
	}
	s_expected = size;
	s_received = 0;
	s_active = true;
	ESP_LOGW(TAG, "receiving %lu bytes", size);
	return "XOB:READY";
}

static const char *ble_ota_finish(void)
{
	if (!s_complete) return "XOB:ERR,INCOMPLETE";
	s_complete = false;
	// Reads the whole image back to check it: about a second.
	esp_err_t err = esp_ota_end(s_handle);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "esp_ota_end failed (%s)", esp_err_to_name(err));
		return err == ESP_ERR_OTA_VALIDATE_FAILED ? "XOB:ERR,IMAGE" : "XOB:ERR,FINISH";
	}

	// An image for another board variant installs and boots, then drives
	// CAN hardware that isn't there: refuse it, as the web upload does.
	char running_variant[16];
	if (app_variant(esp_app_get_description(), running_variant, sizeof(running_variant)))
	{
		esp_app_desc_t new_app;
		char new_variant[16];
		err = esp_ota_get_partition_description(s_partition, &new_app);
		if (err != ESP_OK || !app_variant(&new_app, new_variant, sizeof(new_variant))
				|| strcmp(new_variant, running_variant) != 0)
		{
			ESP_LOGE(TAG, "refusing firmware %.32s: not for this %s board",
					 err == ESP_OK ? new_app.project_name : "(unreadable)", running_variant);
			esp_partition_erase_range(s_partition, 0, s_partition->erase_size);
			return "XOB:ERR,VARIANT";
		}
	}

	err = esp_ota_set_boot_partition(s_partition);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "esp_ota_set_boot_partition failed (%s)", esp_err_to_name(err));
		return "XOB:ERR,FINISH";
	}
	ESP_LOGW(TAG, "image accepted; rebooting");
	schedule_reboot();
	return "XOB:DONE";
}

const char *ble_ota_command(const char *args)
{
	if (args[0] == 0) return "XOB:1";
	if (strcmp(args, "end") == 0) return ble_ota_finish();
	return ble_ota_begin(args);
}

void ble_ota_feed(const uint8_t *data, uint16_t len)
{
	if (!s_active) return;
	if (len > s_expected - s_received)
	{
		esp_ota_abort(s_handle);
		s_active = false;
		ble_ota_notify("XOB:ERR,LENGTH");
		return;
	}
	esp_err_t err = esp_ota_write(s_handle, data, len);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "esp_ota_write failed (%s)", esp_err_to_name(err));
		esp_ota_abort(s_handle);
		s_active = false;
		ble_ota_notify("XOB:ERR,WRITE");
		return;
	}
	s_received += len;
	if (s_received == s_expected)
	{
		// Back to commands: the client sends ATXOBEND next.
		s_complete = true;
		s_active = false;
	}
}

void ble_ota_cancel(void)
{
	if (!s_active) return;
	ESP_LOGW(TAG, "link dropped after %lu of %lu bytes; abandoned",
			 (unsigned long)s_received, (unsigned long)s_expected);
	esp_ota_abort(s_handle);
	s_active = false;
}
