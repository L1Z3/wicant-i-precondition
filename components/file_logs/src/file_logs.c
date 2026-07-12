// Filesystem logging: hooks esp_log's vprintf, buffers formatted lines in a
// RAM ring buffer and flushes them to a rotated log file pair in the
// background. Errors in here are reported with plain printf (straight to
// UART) so the component never logs through the hook it installed.
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdbool.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "file_logs.h"
#include "file_logs_config.h"

#if FILE_LOGS_ENABLE

static vprintf_like_t s_orig_vprintf = NULL;

// Byte ring buffer. Producers are any task inside an ESP_LOGx call, so the
// short copy-in runs under a spinlock; the flush task copies chunks out under
// the same lock and does the slow file I/O outside it.
static char *s_buf = NULL;
static size_t s_buf_size = 0;
static size_t s_head = 0;
static size_t s_tail = 0;
static size_t s_used = 0;
static uint32_t s_dropped = 0;
// Total bytes ever accepted into the ring; with s_used it gives readers an
// absolute stream position that survives eviction (see file_logs_ram_read)
static uint64_t s_total_in = 0;
// RAM-only mode (log to file disabled): no flush task ever drains the ring,
// so ring_write evicts oldest bytes instead of dropping the new line
static bool s_ram_only = false;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static TaskHandle_t s_flush_task = NULL;
// Serializes file access between the flush task and file_logs_flush() callers
static SemaphoreHandle_t s_file_mutex = NULL;
static FILE *s_file = NULL;
static long s_file_size = 0;
static uint32_t s_written = 0;
static uint32_t s_dropped_reported = 0;
static uint32_t s_open_fails = 0;
// Per-file rotation cap; configurable at runtime via file_logs_set_max_total
static long s_max_file_size = FILE_LOGS_MAX_FILE_SIZE;
// Bumped after every completed drain (data on flash, fsync done) so
// file_logs_flush() callers can tell when their request went through
static volatile uint32_t s_flush_cycles = 0;
// Active readers of the log files, guarded by s_file_mutex; rotation is
// deferred while nonzero (littlefs rename/unlink fail on files with open FDs)
static int s_readers = 0;
// Bumped whenever the on-flash stream is rewritten (rotation, deletion) so
// incremental /logs readers know their byte offsets are stale
static uint32_t s_generation = 0;

static void ring_write(const char *data, size_t len)
{
    portENTER_CRITICAL(&s_lock);
    if (!s_buf || len > s_buf_size)
    {
        s_dropped += len;
        portEXIT_CRITICAL(&s_lock);
        return;
    }
    if (len > s_buf_size - s_used)
    {
        if (!s_ram_only)
        {
            s_dropped += len;
            portEXIT_CRITICAL(&s_lock);
            return;
        }
        size_t evict = len - (s_buf_size - s_used);
        s_tail = (s_tail + evict) % s_buf_size;
        s_used -= evict;
    }
    size_t first = s_buf_size - s_head;
    if (first > len)
    {
        first = len;
    }
    memcpy(s_buf + s_head, data, first);
    memcpy(s_buf, data + first, len - first);
    s_head = (s_head + len) % s_buf_size;
    s_used += len;
    s_total_in += len;
    size_t used = s_used;
    portEXIT_CRITICAL(&s_lock);

    // Wake the flush task early once the ring is half full
    if (s_flush_task && used >= s_buf_size / 2)
    {
        xTaskNotifyGive(s_flush_task);
    }
}

// Drops ANSI color escapes (ESC ... m) so the file stays plain text even if
// CONFIG_LOG_COLORS is on. Returns the new length.
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

static int file_logs_vprintf(const char *fmt, va_list args)
{
    char line[FILE_LOGS_MAX_LINE];
    va_list copy;
    va_copy(copy, args);
    int n = vsnprintf(line, sizeof(line), fmt, copy);
    va_end(copy);

    if (n > 0)
    {
        size_t len = (size_t)n;
        if (len >= sizeof(line))
        {
            // Truncated: the trailing newline was lost, put one back
            len = sizeof(line) - 1;
            line[len - 1] = '\n';
        }
        len = strip_ansi(line, len);
        ring_write(line, len);
    }

    return s_orig_vprintf ? s_orig_vprintf(fmt, args) : vprintf(fmt, args);
}

void file_logs_early_init(void)
{
    if (s_buf)
    {
        return;
    }

    size_t size = FILE_LOGS_BUF_SIZE;
    char *buf = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf)
    {
        size = FILE_LOGS_BUF_SIZE_FALLBACK;
        buf = malloc(size);
    }
    if (!buf)
    {
        printf("[file_logs] buffer alloc failed, filesystem logging disabled\n");
        return;
    }

    s_buf_size = size;
    s_buf = buf;
    s_orig_vprintf = esp_log_set_vprintf(file_logs_vprintf);
}

void file_logs_set_max_total(uint32_t total_bytes)
{
    // Two rotation files share the budget; refuse caps too small for even a
    // few flush cycles worth of logs
    if (total_bytes / 2 >= 8 * 1024)
    {
        s_max_file_size = (long)(total_bytes / 2);
    }
}

void file_logs_ram_only(void)
{
    portENTER_CRITICAL(&s_lock);
    s_ram_only = true;
    portEXIT_CRITICAL(&s_lock);
}

bool file_logs_is_ram_only(void)
{
    return s_ram_only;
}

uint32_t file_logs_generation(void)
{
    return s_generation;
}

uint64_t file_logs_ram_total(void)
{
    portENTER_CRITICAL(&s_lock);
    uint64_t total = s_total_in;
    portEXIT_CRITICAL(&s_lock);
    return total;
}

size_t file_logs_ram_read(uint64_t *pos, char *dst, size_t dst_size)
{
    portENTER_CRITICAL(&s_lock);
    if (!s_buf)
    {
        portEXIT_CRITICAL(&s_lock);
        return 0;
    }
    uint64_t earliest = s_total_in - s_used;
    if (*pos < earliest)
    {
        *pos = earliest;
    }
    size_t avail = (size_t)(s_total_in - *pos);
    size_t len = avail < dst_size ? avail : dst_size;
    size_t start = (s_tail + (size_t)(*pos - earliest)) % s_buf_size;
    size_t first = s_buf_size - start;
    if (first > len)
    {
        first = len;
    }
    memcpy(dst, s_buf + start, first);
    memcpy(dst + first, s_buf, len - first);
    portEXIT_CRITICAL(&s_lock);
    *pos += len;
    return len;
}

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

// Must be called with s_file_mutex held
static bool log_file_open(void)
{
    if (s_file)
    {
        return true;
    }
    s_file = fopen(FILE_LOGS_CUR_PATH, "a");
    if (!s_file)
    {
        s_open_fails++;
        if (s_open_fails < 4 || (s_open_fails & 0x3F) == 0)
        {
            printf("[file_logs] fopen %s failed (attempt %lu)\n",
                   FILE_LOGS_CUR_PATH, (unsigned long)s_open_fails);
        }
        return false;
    }
    fseek(s_file, 0, SEEK_END);
    s_file_size = ftell(s_file);
    if (s_file_size < 0)
    {
        s_file_size = 0;
    }
    return true;
}

// Must be called with s_file_mutex held
static void log_file_rotate_if_needed(void)
{
    if (!s_file || s_file_size < s_max_file_size)
    {
        return;
    }
    if (s_readers > 0)
    {
        // A reader is streaming the files; retry on a later drain cycle
        return;
    }
    fclose(s_file);
    s_file = NULL;
    s_file_size = 0;
    unlink(FILE_LOGS_OLD_PATH);
    if (rename(FILE_LOGS_CUR_PATH, FILE_LOGS_OLD_PATH) != 0)
    {
        // Rename failed: truncate in place rather than growing without bound
        unlink(FILE_LOGS_CUR_PATH);
    }
    s_generation++;
}

// Must be called with s_file_mutex held. Copies ring contents out in chunks
// and appends them to the current log file.
static void drain_to_file(void)
{
    if (!log_file_open())
    {
        return;
    }

    char chunk[512];
    size_t total_written = 0;
    for (;;)
    {
        portENTER_CRITICAL(&s_lock);
        size_t len = s_used;
        if (len > sizeof(chunk))
        {
            len = sizeof(chunk);
        }
        size_t first = s_buf_size - s_tail;
        if (first > len)
        {
            first = len;
        }
        memcpy(chunk, s_buf + s_tail, first);
        memcpy(chunk + first, s_buf, len - first);
        s_tail = (s_tail + len) % s_buf_size;
        s_used -= len;
        portEXIT_CRITICAL(&s_lock);

        if (len == 0)
        {
            break;
        }
        if (fwrite(chunk, 1, len, s_file) != len)
        {
            // Filesystem full or write error: drop this chunk and back off
            // until the next cycle
            break;
        }
        total_written += len;
    }

    uint32_t dropped = s_dropped;
    if (dropped != s_dropped_reported)
    {
        int n = fprintf(s_file, "[file_logs] dropped %lu bytes (ring full)\n",
                        (unsigned long)(dropped - s_dropped_reported));
        s_dropped_reported = dropped;
        if (n > 0)
        {
            total_written += (size_t)n;
        }
    }

    if (total_written > 0)
    {
        fflush(s_file);
        fsync(fileno(s_file));
        s_file_size += total_written;
        s_written += total_written;
    }
    log_file_rotate_if_needed();
    s_flush_cycles++;
}

static void drain_locked(void)
{
    if (xSemaphoreTake(s_file_mutex, pdMS_TO_TICKS(2000)) != pdTRUE)
    {
        return;
    }
    drain_to_file();
    xSemaphoreGive(s_file_mutex);
}

// File I/O always runs on the flush task's stack: callers from other tasks
// (some with small stacks, e.g. the timer service task pre-restart) just
// poke the flush task and wait for the ring to hit flash
void file_logs_flush(void)
{
    if (!s_file_mutex)
    {
        return;
    }
    if (!s_flush_task || xTaskGetCurrentTaskHandle() == s_flush_task)
    {
        drain_locked();
        return;
    }

    uint32_t start = s_flush_cycles;
    xTaskNotifyGive(s_flush_task);
    for (int i = 0; i < 200; i++)
    {
        portENTER_CRITICAL(&s_lock);
        size_t used = s_used;
        portEXIT_CRITICAL(&s_lock);
        if (s_flush_cycles != start && used == 0)
        {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void file_logs_task(void *arg)
{
    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(FILE_LOGS_FLUSH_INTERVAL_MS));
        drain_locked();
    }
}

void file_logs_start(void)
{
    if (!s_buf || s_flush_task)
    {
        return;
    }

    s_file_mutex = xSemaphoreCreateMutex();
    if (!s_file_mutex)
    {
        return;
    }

    // Boot marker so consecutive boots are easy to tell apart in the file
    if (xSemaphoreTake(s_file_mutex, portMAX_DELAY) == pdTRUE)
    {
        if (log_file_open())
        {
            int n = fprintf(s_file, "\n==== boot (reset: %s, fw: %s) ====\n",
                            reset_reason_str(esp_reset_reason()), GIT_SHA);
            if (n > 0)
            {
                s_file_size += n;
                s_written += (uint32_t)n;
            }
        }
        xSemaphoreGive(s_file_mutex);
    }

    if (xTaskCreate(file_logs_task, "file_logs", 4096, NULL, 3, &s_flush_task) != pdPASS)
    {
        printf("[file_logs] task create failed\n");
        s_flush_task = NULL;
    }
}

bool file_logs_delete(void)
{
    if (!s_file_mutex)
    {
        // Not started: nothing holds the files open
        unlink(FILE_LOGS_CUR_PATH);
        unlink(FILE_LOGS_OLD_PATH);
        s_generation++;
        return true;
    }
    if (xSemaphoreTake(s_file_mutex, pdMS_TO_TICKS(5000)) != pdTRUE)
    {
        return false;
    }
    bool ok = false;
    if (s_readers == 0)
    {
        // Close our append FD first: littlefs cannot unlink an open file
        if (s_file)
        {
            fclose(s_file);
            s_file = NULL;
            s_file_size = 0;
        }
        // Missing files are fine, ignore unlink errors
        unlink(FILE_LOGS_CUR_PATH);
        unlink(FILE_LOGS_OLD_PATH);
        s_generation++;
        ok = true;
    }
    xSemaphoreGive(s_file_mutex);
    return ok;
}

bool file_logs_read_begin(void)
{
    if (!s_file_mutex)
    {
        // Not started yet: no rotation to race with
        return true;
    }
    if (xSemaphoreTake(s_file_mutex, pdMS_TO_TICKS(5000)) != pdTRUE)
    {
        return false;
    }
    s_readers++;
    xSemaphoreGive(s_file_mutex);
    return true;
}

void file_logs_read_end(void)
{
    if (!s_file_mutex)
    {
        return;
    }
    if (xSemaphoreTake(s_file_mutex, portMAX_DELAY) == pdTRUE)
    {
        if (s_readers > 0)
        {
            s_readers--;
        }
        xSemaphoreGive(s_file_mutex);
    }
}

void file_logs_get_stats(uint32_t *buffered, uint32_t *dropped, uint32_t *written)
{
    portENTER_CRITICAL(&s_lock);
    if (buffered)
    {
        *buffered = (uint32_t)s_used;
    }
    if (dropped)
    {
        *dropped = s_dropped;
    }
    portEXIT_CRITICAL(&s_lock);
    if (written)
    {
        *written = s_written;
    }
}

#else // FILE_LOGS_ENABLE

void file_logs_early_init(void) {}
void file_logs_start(void) {}
void file_logs_set_max_total(uint32_t total_bytes) { (void)total_bytes; }
void file_logs_ram_only(void) {}
bool file_logs_is_ram_only(void) { return false; }
uint32_t file_logs_generation(void) { return 0; }
uint64_t file_logs_ram_total(void) { return 0; }
size_t file_logs_ram_read(uint64_t *pos, char *dst, size_t dst_size)
{
    (void)pos;
    (void)dst;
    (void)dst_size;
    return 0;
}
bool file_logs_delete(void) { return true; }
void file_logs_flush(void) {}
bool file_logs_read_begin(void) { return true; }
void file_logs_read_end(void) {}
void file_logs_get_stats(uint32_t *buffered, uint32_t *dropped, uint32_t *written)
{
    if (buffered) *buffered = 0;
    if (dropped) *dropped = 0;
    if (written) *written = 0;
}

#endif // FILE_LOGS_ENABLE
