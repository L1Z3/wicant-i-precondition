#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Captures the ESP_LOG stream to the filesystem. A vprintf hook copies every
// formatted log line into a RAM ring buffer; a background task later drains
// the ring to FILE_LOGS_CUR_PATH, rotating to FILE_LOGS_OLD_PATH. UART log
// output is unaffected. Lines that arrive while the ring is full are dropped
// (drop-newest, so early-boot logs survive) and accounted for with a marker
// line in the file.

// Call as early as possible (first thing in app_main): allocates the ring
// and hooks esp_log so everything from that point on is captured.
void file_logs_early_init(void);

// Call once the filesystem is mounted: writes a boot marker and starts the
// background flush task.
void file_logs_start(void);

// Synchronous best-effort flush of whatever is buffered; call right before
// esp_restart() so the tail of the log survives the reboot.
void file_logs_flush(void);

// Counters for troubleshooting: bytes currently buffered, bytes dropped on a
// full ring, bytes written to the filesystem since boot.
void file_logs_get_stats(uint32_t *buffered, uint32_t *dropped, uint32_t *written);

#ifdef __cplusplus
}
#endif
