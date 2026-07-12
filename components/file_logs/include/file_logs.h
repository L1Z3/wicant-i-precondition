#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Captures the ESP_LOG stream to the filesystem. A vprintf hook copies every
// formatted log line into a RAM ring buffer; a background task later drains
// the ring to FILE_LOGS_CUR_PATH, rotating to FILE_LOGS_OLD_PATH. UART log
// output is unaffected. In file mode, lines that arrive while the ring is
// full are dropped (drop-newest, so early-boot logs survive) and accounted
// for with a marker line in the file; in RAM-only mode the ring wraps,
// evicting the oldest bytes instead.

// Call as early as possible (first thing in app_main): allocates the ring
// and hooks esp_log so everything from that point on is captured.
void file_logs_early_init(void);

// Call once the filesystem is mounted: writes a boot marker and starts the
// background flush task.
void file_logs_start(void);

// Sets the total on-flash log budget (both rotation files combined, so each
// file is capped at half). Values too small to be useful are ignored. Call
// before file_logs_start(); defaults to FILE_LOGS_MAX_FILE_SIZE * 2.
void file_logs_set_max_total(uint32_t total_bytes);

// Switches to RAM-only logging. Meant for the log-to-file-disabled config,
// called instead of file_logs_start(): the hook and ring stay active but no
// flush task runs, and the ring becomes a circular in-RAM log (oldest bytes
// evicted once full) readable via file_logs_ram_read. Existing log files are
// untouched.
void file_logs_ram_only(void);

// True after file_logs_ram_only() put the component in RAM-only mode.
bool file_logs_is_ram_only(void);

// Copies up to dst_size bytes of the in-RAM log into dst, returning the
// number of bytes copied (0 when caught up). *pos is an absolute stream
// offset maintained by this function: start at 0 and pass the same variable
// on every call; positions already evicted from the ring snap forward to the
// oldest data still available.
size_t file_logs_ram_read(uint64_t *pos, char *dst, size_t dst_size);

// Synchronous best-effort flush of whatever is buffered; call right before
// esp_restart() so the tail of the log survives the reboot.
void file_logs_flush(void);

// Deletes both log files from the filesystem (buffered-but-unflushed lines
// survive and start a fresh file on the next flush). Returns false if the
// files are busy: a /logs reader is streaming them or the file mutex timed out.
bool file_logs_delete(void);

// Readers of the log files (e.g. the /logs HTTP handler) must bracket their
// open..close with these: rotation is deferred while any reader is active,
// since littlefs cannot rename/unlink a file that has an open FD. read_begin
// returns false on timeout; only call read_end if it returned true.
bool file_logs_read_begin(void);
void file_logs_read_end(void);

// Counters for troubleshooting: bytes currently buffered, bytes dropped on a
// full ring, bytes written to the filesystem since boot.
void file_logs_get_stats(uint32_t *buffered, uint32_t *dropped, uint32_t *written);

#ifdef __cplusplus
}
#endif
