#ifndef WICAN_BEEP_H
#define WICAN_BEEP_H

#include <stdbool.h>
#include <stdint.h>

// Initialize once after configuration is loaded, before callers start queuing
// sounds. A dedicated worker owns playback; no tick or popup is required.
void beep_init(void);

// Queue 1..255 head-unit beeps without waiting for playback. Safe to call from
// tasks; requests play in FIFO order without interleaving. Returns false if
// uninitialized, count is zero, or the queue is full. CAN delivery is best effort.
// Timing is configured by BEEP_INTERVAL_MS in beep.c.
bool beep_play(uint8_t count);

#endif
