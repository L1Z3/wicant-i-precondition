#pragma once

#include "sdkconfig.h"
#include "hw_config.h"

// Simple compile-time configuration (no Kconfig), same pattern as debug_logs.
// Adjust these defines as needed or override via compiler -D flags.

#ifndef FILE_LOGS_ENABLE
#define FILE_LOGS_ENABLE 1
#endif

// Directory the log files live in. The app must mount a writable filesystem
// here before calling file_logs_start().
#ifndef FILE_LOGS_DIR
#define FILE_LOGS_DIR FS_MOUNT_POINT
#endif

#define FILE_LOGS_CUR_PATH FILE_LOGS_DIR "/wican.log"
#define FILE_LOGS_OLD_PATH FILE_LOGS_DIR "/wican.old.log"

// RAM ring buffer that holds log lines from early boot until the filesystem
// is mounted, and batches them between flushes afterwards. Lives in PSRAM
// where available; the V300 (ESP32-C3) has none, so it gets a small
// internal-RAM buffer instead.
#ifndef FILE_LOGS_BUF_SIZE
#ifdef CONFIG_SPIRAM
#define FILE_LOGS_BUF_SIZE (128 * 1024)
#else
#define FILE_LOGS_BUF_SIZE (16 * 1024)
#endif
#endif

// Fallback buffer size if the PSRAM allocation fails.
#ifndef FILE_LOGS_BUF_SIZE_FALLBACK
#define FILE_LOGS_BUF_SIZE_FALLBACK (16 * 1024)
#endif

// A single formatted log line is truncated to this many bytes.
#ifndef FILE_LOGS_MAX_LINE
#define FILE_LOGS_MAX_LINE 256
#endif

// The flush task drains the ring to flash on this interval, or earlier if the
// ring crosses half full. Bigger = fewer flash writes, more loss on a crash.
#ifndef FILE_LOGS_FLUSH_INTERVAL_MS
#define FILE_LOGS_FLUSH_INTERVAL_MS 10000
#endif

// Rotation limit for wican.log: past this size it is renamed to
// wican.old.log, so logs occupy at most twice this on the storage partition.
// Sized off the board's storage partition, which also holds the config
// files: roomy on 1M+ partitions, conservative on small ones (V300: 300K).
#ifndef FILE_LOGS_MAX_FILE_SIZE
#ifndef HW_STORAGE_PARTITION_SIZE
#error "HW_STORAGE_PARTITION_SIZE not defined; expected from hw_config.h"
#endif
#if HW_STORAGE_PARTITION_SIZE >= (1024 * 1024)
#define FILE_LOGS_MAX_FILE_SIZE (512 * 1024)
#else
#define FILE_LOGS_MAX_FILE_SIZE (32 * 1024)
#endif
#endif
