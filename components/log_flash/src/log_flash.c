#include "log_flash.h"

// Nested regions are possible (a drain can rotate inside its own mark), so
// keep a depth and a monotonically increasing sequence. These are read from
// another core by the SPI diagnostic path; 32-bit aligned loads/stores are
// sufficient for this diagnostic.
static volatile int s_depth;
static volatile uint32_t s_seq;

void log_flash_begin(void)
{
    s_depth++;
    s_seq++;
}

void log_flash_end(void)
{
    if (s_depth > 0)
    {
        s_depth--;
    }
}

bool log_flash_active(void)
{
    return s_depth > 0;
}

uint32_t log_flash_seq(void)
{
    return s_seq;
}
