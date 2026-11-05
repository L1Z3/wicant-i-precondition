// The RAM side of logging: every esp_log line is also appended to a byte ring
// that an HTTP reader can follow while the device writes.  There are no log
// files and no flash writes here; the ring is the history, and once it is full
// the oldest bytes are overwritten (counted in the "dropped" statistic).
#include "log_stream.h"
#include "log_stream_config.h"

#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"

static char *s_buf;
static uint32_t s_size;      // ring capacity, 0 when allocation failed
static uint32_t s_head;      // next byte to write
static uint32_t s_tail;      // oldest byte
static uint32_t s_used;
static uint64_t s_total_in;  // bytes ever written; also the absolute position
static uint32_t s_dropped;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static vprintf_like_t s_console_vprintf;

static const char *reset_reason_str(esp_reset_reason_t r)
{
    switch (r)
    {
        case ESP_RST_POWERON: return "poweron";
        case ESP_RST_EXT: return "external";
        case ESP_RST_SW: return "software";
        case ESP_RST_PANIC: return "panic";
        case ESP_RST_INT_WDT: return "int_wdt";
        case ESP_RST_TASK_WDT: return "task_wdt";
        case ESP_RST_WDT: return "other_wdt";
        case ESP_RST_DEEPSLEEP: return "deepsleep";
        case ESP_RST_BROWNOUT: return "brownout";
        case ESP_RST_SDIO: return "sdio";
        default: return "unknown";
    }
}

// esp_log lines can carry colour sequences; the ring is plain text.
static size_t strip_ansi(char *buf, size_t len)
{
    size_t out = 0;
    bool in_esc = false;
    for (size_t i = 0; i < len; i++)
    {
        char c = buf[i];
        if (in_esc)
        {
            if (c == 'm')
            {
                in_esc = false;
            }
            continue;
        }
        if (c == '\033')
        {
            in_esc = true;
            continue;
        }
        buf[out++] = c;
    }
    return out;
}

static void ring_write(const char *src, size_t len)
{
    if (len == 0)
    {
        return;
    }

    portENTER_CRITICAL(&s_lock);
    if (s_size == 0)
    {
        s_dropped += (uint32_t)len;
        portEXIT_CRITICAL(&s_lock);
        return;
    }
    if (len > s_size)
    {
        // Keep the tail of an oversized write: that is the newest text.
        s_dropped += (uint32_t)(len - s_size);
        src += len - s_size;
        len = s_size;
    }
    while (len > 0)
    {
        if (s_used == s_size)
        {
            s_tail = (s_tail + 1) % s_size;
            s_used--;
            s_dropped++;
        }
        uint32_t chunk = s_size - s_used;
        if (chunk > len)
        {
            chunk = (uint32_t)len;
        }
        uint32_t first = s_size - s_head;
        if (chunk > first)
        {
            chunk = first;
        }
        memcpy(&s_buf[s_head], src, chunk);
        s_head = (s_head + chunk) % s_size;
        s_used += chunk;
        s_total_in += chunk;
        src += chunk;
        len -= chunk;
    }
    portEXIT_CRITICAL(&s_lock);
}

static int log_stream_vprintf(const char *fmt, va_list args)
{
    char line[LOG_STREAM_LINE_SIZE];
    va_list copy;
    va_copy(copy, args);
    int n = vsnprintf(line, sizeof(line), fmt, copy);
    va_end(copy);
    if (n > 0)
    {
        size_t len = (size_t)n < sizeof(line) ? (size_t)n : sizeof(line) - 1;
        len = strip_ansi(line, len);
        ring_write(line, len);
    }
    return s_console_vprintf ? s_console_vprintf(fmt, args) : vprintf(fmt, args);
}

void log_stream_init(void)
{
#if CONFIG_SPIRAM
    s_buf = heap_caps_malloc(LOG_STREAM_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_buf)
    {
        s_size = LOG_STREAM_BUF_SIZE;
    }
#endif
    if (!s_buf)
    {
        s_buf = malloc(LOG_STREAM_FALLBACK_BUF_SIZE);
        if (s_buf)
        {
            s_size = LOG_STREAM_FALLBACK_BUF_SIZE;
        }
    }

    char hdr[96];
    int n = snprintf(hdr, sizeof(hdr), "\n==== boot (reset: %s, fw: %s) ====\n",
                     reset_reason_str(esp_reset_reason()), esp_app_get_description()->version);
    if (n > 0)
    {
        ring_write(hdr, (size_t)n < sizeof(hdr) ? (size_t)n : sizeof(hdr) - 1);
    }

    s_console_vprintf = esp_log_set_vprintf(log_stream_vprintf);
}

uint64_t log_stream_total(void)
{
    portENTER_CRITICAL(&s_lock);
    uint64_t total = s_total_in;
    portEXIT_CRITICAL(&s_lock);
    return total;
}

size_t log_stream_read(uint64_t *pos, char *dst, size_t dst_size)
{
    if (pos == NULL || dst == NULL || dst_size == 0)
    {
        return 0;
    }

    portENTER_CRITICAL(&s_lock);
    if (s_size == 0)
    {
        *pos = s_total_in;
        portEXIT_CRITICAL(&s_lock);
        return 0;
    }
    uint64_t earliest = s_total_in - s_used;
    if (*pos < earliest)
    {
        *pos = earliest;
    }
    if (*pos > s_total_in)
    {
        *pos = s_total_in;
    }

    size_t n = (size_t)(s_total_in - *pos);
    if (n > dst_size)
    {
        n = dst_size;
    }
    if (n > 0)
    {
        size_t start = (s_tail + (size_t)(*pos - earliest)) % s_size;
        size_t first = s_size - start;
        if (first > n)
        {
            first = n;
        }
        memcpy(dst, &s_buf[start], first);
        if (n > first)
        {
            memcpy(dst + first, s_buf, n - first);
        }
        *pos += n;
    }
    portEXIT_CRITICAL(&s_lock);
    return n;
}

size_t log_stream_size(void)
{
    return s_size;
}

void log_stream_get_stats(uint32_t *buffered, uint32_t *dropped)
{
    portENTER_CRITICAL(&s_lock);
    if (buffered)
    {
        *buffered = s_used;
    }
    if (dropped)
    {
        *dropped = s_dropped;
    }
    portEXIT_CRITICAL(&s_lock);
}
