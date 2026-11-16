#pragma once

#include "sdkconfig.h"

// Ring used by the streaming log.  PSRAM first; a smaller internal-RAM ring is
// the fallback, and if that also fails logs still go to the console.
#define LOG_STREAM_BUF_SIZE          (128 * 1024)

#if CONFIG_IDF_TARGET_ESP32C3
// The C3 has no PSRAM, so this fallback is its only ring, and it comes out of
// the ~29 KB of internal heap left after WiFi and lwIP start.  Measured with a
// 16 KB ring: 60 s after boot the board reported heap free=5148 with a largest
// block of 1408, and the first GET / of the config page died in
// httpd_resp_send ("httpd_sock_err: error in send : 11" after 5 s) and took
// the web server with it.  4 KB returns 12,288 B and still holds the last ~30
// log lines.
#define LOG_STREAM_FALLBACK_BUF_SIZE (4 * 1024)
#else
#define LOG_STREAM_FALLBACK_BUF_SIZE (16 * 1024)
#endif

// Longest line kept.  Longer lines are truncated; esp_log lines are far below
// this in practice.
#define LOG_STREAM_LINE_SIZE 256
