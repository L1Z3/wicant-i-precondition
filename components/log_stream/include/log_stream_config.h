#pragma once

// Ring used by the streaming log.  PSRAM first; a smaller internal-RAM ring is
// the fallback, and if that also fails logs still go to the console.
#define LOG_STREAM_BUF_SIZE          (128 * 1024)
#define LOG_STREAM_FALLBACK_BUF_SIZE (16 * 1024)

// Longest line kept.  Longer lines are truncated; esp_log lines are far below
// this in practice.
#define LOG_STREAM_LINE_SIZE 256
