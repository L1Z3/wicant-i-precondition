/*
 * Firmware update over BLE; see ble_ota.c.
 */
#ifndef __BLE_OTA_H__
#define __BLE_OTA_H__

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// The BLE TX queue, for the unsolicited error line. main.c, once at boot.
void ble_ota_set_tx_queue(QueueHandle_t *queue);

// ATXOB<args>, with args the text after "xob": "", "<size>" or "end".
// Returns the reply, or NULL for a malformed command.
const char *ble_ota_command(const char *args);

// True while image bytes are expected: ble.c sends every write on the
// command characteristic to ble_ota_feed instead of the ELM327 parser.
bool ble_ota_active(void);

// One BLE write of image bytes. BTC task only.
void ble_ota_feed(const uint8_t *data, uint16_t len);

// The BLE link dropped: abandon a transfer in progress. BTC task only.
void ble_ota_cancel(void);

#endif
