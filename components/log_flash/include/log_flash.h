#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Marks a region that may write or erase the log filesystem. This is
// deliberately a tiny, dependency-free component so both file_logs and the
// MCP2518FD SPI shim can use it without pulling in each other.
//
// The SPI LL path samples these values around a transfer. If a write starts
// or ends while the bus is polling, the slow-transfer log can say so; a
// delay with no log_flash activity points at Wi-Fi preemption or another
// flash user instead.
void log_flash_begin(void);
void log_flash_end(void);
bool log_flash_active(void);
uint32_t log_flash_seq(void);

#ifdef __cplusplus
}
#endif
