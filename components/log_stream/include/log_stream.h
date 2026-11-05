// Streaming log for the device: esp_log lines are also kept in a RAM ring that
// can be read over HTTP while they are produced.  Nothing is written to flash;
// the ring is the whole history and the oldest bytes are overwritten once it is
// full.  See the /logs handler in config_server.c for the reader protocol.
#pragma once

#include <stddef.h>
#include <stdint.h>

// Hook esp_log and allocate the ring.  Call once, as early as possible: lines
// logged before this are not captured (the hook does not exist yet).
void log_stream_init(void);

// Absolute position of the next byte that will be written.  Valid positions for
// reading are [total - buffered, total].
uint64_t log_stream_total(void);

// Copy the bytes at *pos into dst.  *pos is snapped forward to the oldest byte
// still buffered when it is too old.  Returns the number of bytes copied (0 when
// the caller has caught up) and advances *pos by that much.
size_t log_stream_read(uint64_t *pos, char *dst, size_t dst_size);

// Size of the ring in bytes (0 when allocation failed).
size_t log_stream_size(void);

// Bytes currently buffered and bytes lost because the ring was full.
void log_stream_get_stats(uint32_t *buffered, uint32_t *dropped);
