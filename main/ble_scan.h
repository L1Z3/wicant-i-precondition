#ifndef __BLE_SCAN_H__
#define __BLE_SCAN_H__

#include <stdbool.h>
#include <stdint.h>

/*
 * Runs a blocking BLE GAP scan for `ms` milliseconds and returns a malloc'd
 * JSON string (caller frees). `name` and `uuid` are optional filters; either
 * one (or both) restricts the returned devices.
 *
 * The module owns the Bluetooth controller + Bluedroid stack. It refuses to
 * start when the WiCAN GATT server is enabled (`ble_status=enable`), because
 * main/ble.c owns the single GAP callback for that stack.
 */
char *ble_scan_json(uint32_t ms, const char *name, const char *uuid);

bool ble_scan_stack_ready(void);

#endif
