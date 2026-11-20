#ifndef __BLE_BUTTON_H__
#define __BLE_BUTTON_H__

#include <stdbool.h>

/*
 * BLE button listener.
 *
 * Associates with a device that advertises service UUID 0xFFE3 (the generic
 * iTag/selfie-button service), connects as a GATT client, subscribes to every
 * notify/indicate characteristic in that service, and calls
 * precondition_toggle_request() on the first notification of a press.
 *
 * Association is by service UUID, not by a hard-coded MAC, so another device
 * with the same chip/firmware works without reflashing.  The last associated
 * address is stored in NVS and preferred on the next boot; if it is absent the
 * listener falls back to any 0xFFE3 device and re-binds it.
 */
bool ble_button_init(void);

void ble_button_set_paused(bool paused);
bool ble_button_paused(void);

/* Start a fresh association: forget the stored address and bind the next
 * 0xFFE3 device seen. */
void ble_button_learn(void);

/* Forget the stored address and disconnect. */
void ble_button_forget(void);

/* malloc'd JSON; caller frees. */
char *ble_button_status_json(void);

#endif
