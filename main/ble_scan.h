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

/* Bring up the controller + Bluedroid and the GAP scan callback once. */
bool ble_scan_ensure_stack(void);

typedef struct {
    uint8_t bda[6];
    uint8_t addr_type;
    int rssi;
    const uint8_t *adv;
    uint8_t adv_len;
    const uint8_t *rsp;
    uint8_t rsp_len;
    const char *name;
    const char *uuid;
} ble_scan_report_t;

typedef void (*ble_scan_report_cb_t)(const ble_scan_report_t *rpt, void *ctx);

/*
 * Start an asynchronous scan (duration in seconds) and deliver every report to
 * `cb` from the Bluetooth callback context.  The observer must be quick and
 * must not call ble_scan_stop(); the owner stops the scan after its window.
 * Returns false if the stack is unavailable or another scan is already active.
 */
bool ble_scan_start(uint32_t seconds, ble_scan_report_cb_t cb, void *ctx);
void ble_scan_stop(void);

#endif
