// Drive the SLCAN parser with the chunking the host-facing transports really
// produce.
//
// slcan_parse_str() appends each received chunk to a heap buffer and is allowed
// to see a command split across chunks: the TCP rx task hands it whatever
// recv() returned (up to DEV_BUFFER_LENGTH - 1 = 64 bytes), and /ws hands it
// one message per call. A burst of several short frames therefore arrives as a
// few 64-byte chunks whose boundaries fall in the middle of a line.
//
// The parser keeps its command state in statics across calls, so it must
// continue at the byte where the previous call stopped, not at byte 0.
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/wait.h>
#include "test_support.h"
#include "can.h"
#include "driver/twai.h"
#include "freertos/queue.h"

#include "slcan.c"

typedef struct {
    can_bus_t bus;
    twai_message_t frame;
} sent_t;

static sent_t sent[32];
static size_t sent_count;

static char response[64];
static size_t response_count;

// --- fakes the parser calls -------------------------------------------------
esp_err_t can_send(can_bus_t bus, twai_message_t *frame, TickType_t wait) {
    (void)wait;
    if (sent_count < sizeof(sent) / sizeof(sent[0])) {
        sent[sent_count++] = (sent_t){bus, *frame};
    }
    return 0;
}

void can_enable(can_bus_t bus) { (void)bus; }
void can_disable(can_bus_t bus) { (void)bus; }
void can_set_silent(can_bus_t bus, uint8_t flag) { (void)bus; (void)flag; }
void can_set_loopback(can_bus_t bus, uint8_t flag) { (void)bus; (void)flag; }
void can_set_bitrate(can_bus_t bus, uint8_t rate) { (void)bus; (void)rate; }
void can_set_filter(can_bus_t bus, uint32_t f) { (void)bus; (void)f; }
void can_set_mask(can_bus_t bus, uint32_t m) { (void)bus; (void)m; }

static void record_response(char *str, uint32_t len, QueueHandle_t *q) {
    (void)q;
    size_t n = len ? len : strlen(str);
    if (n > sizeof(response) - 1) {
        n = sizeof(response) - 1;
    }
    if (response_count + n < sizeof(response)) {
        memcpy(response + response_count, str, n);
        response_count += n;
    }
}

// --- helpers ----------------------------------------------------------------
// Build one 11-bit data-frame line: 't' <3 hex id> <dlc hex> <2*dlc data hex>
// '\r'. Building it rather than hard-coding a literal keeps the length honest:
// a hand-written literal is easy to leave one byte short, and a short line
// silently drops the terminator and swallows the next command.
static size_t mk_std_line(char *out, uint32_t id, uint8_t dlc, const uint8_t *data) {
    static const char hex[] = "0123456789ABCDEF";
    size_t n = 0;
    out[n++] = 't';
    out[n++] = hex[(id >> 8) & 0xF];
    out[n++] = hex[(id >> 4) & 0xF];
    out[n++] = hex[id & 0xF];
    out[n++] = hex[dlc & 0xF];
    for (uint8_t b = 0; b < dlc; b++) {
        out[n++] = hex[(data[b] >> 4) & 0xF];
        out[n++] = hex[data[b] & 0xF];
    }
    out[n++] = '\r';
    return n;
}

// The firmware hands the parser a twai_message_t it reuses across calls, so
// keep one too: a command split across two calls resumes mid-frame.
static twai_message_t frame;

static void feed(const char *s, size_t len, int64_t at_us) {
    fake_now = at_us;
    slcan_parse_str((uint8_t *)s, (uint8_t)len, &frame, NULL);
}

static void check_frame(size_t i, uint32_t id, const uint8_t *data, uint8_t dlc) {
    CHECK_MSG(i < sent_count, "expected at least %zu frames, got %zu", i + 1, sent_count);
    if (i >= sent_count) {
        return;
    }
    CHECK_MSG(sent[i].bus == CAN_BUS_0, "frame %zu went to bus %d", i, sent[i].bus);
    CHECK_MSG(sent[i].frame.identifier == id,
              "frame %zu id 0x%x, want 0x%x", i, (unsigned)sent[i].frame.identifier, (unsigned)id);
    CHECK_MSG(sent[i].frame.data_length_code == dlc,
              "frame %zu dlc %u, want %u", i, sent[i].frame.data_length_code, dlc);
    for (uint8_t b = 0; b < dlc && b < 8; b++) {
        CHECK_MSG(sent[i].frame.data[b] == data[b],
                  "frame %zu data[%u] 0x%02x, want 0x%02x",
                  i, b, sent[i].frame.data[b], data[b]);
    }
}

static const uint8_t D4F1[8] = {0x00, 0xD0, 0, 0, 0, 0, 0, 0};
static const uint8_t D4A2[8] = {0x00, 0x00, 0xFF, 0x03, 0xFF, 0xFF, 0x00, 0x00};

static void reset(void) {
    sent_count = 0;
    response_count = 0;
    memset(sent, 0, sizeof(sent));
    memset(response, 0, sizeof(response));
    memset(&frame, 0, sizeof(frame));
    fake_now = 0;
    slcan_init(record_response);
}

// --- suites -----------------------------------------------------------------
// Each line arrives as its own call, one command per TCP segment, 40 ms apart.
static void run_complete_lines(void) {
    reset();
    char line[32];
    size_t len = mk_std_line(line, 0x4F1, 8, D4F1);
    for (int i = 0; i < 3; i++) {
        feed(line, len, (int64_t)(i + 1) * 40000);
    }
    CHECK_MSG(sent_count == 3, "three complete lines 40 ms apart sent %zu frames", sent_count);
    for (size_t i = 0; i < 3 && i < sent_count; i++) {
        check_frame(i, 0x4F1, D4F1, 8);
    }
}

// All three lines in one call, as when the host writes the whole burst at once.
static void run_burst_one_call(void) {
    reset();
    char buf[128];
    size_t n = 0;
    for (int i = 0; i < 3; i++) {
        n += mk_std_line(buf + n, 0x4F1, 8, D4F1);
    }
    feed(buf, n, 40000);
    CHECK_MSG(sent_count == 3, "one-call burst sent %zu frames", sent_count);
    for (size_t i = 0; i < 3 && i < sent_count; i++) {
        check_frame(i, 0x4F1, D4F1, 8);
    }
}

// Six lines delivered in the two chunks the TCP rx task would hand over (it
// reads at most DEV_BUFFER_LENGTH - 1 = 64 bytes). The boundary lands inside a
// line, so the parser resumes in SL_FRAME_DATA on the second call.
static void run_recv_chunked(void) {
    reset();
    char buf[256];
    size_t n = 0;
    for (int i = 0; i < 3; i++) {
        n += mk_std_line(buf + n, 0x4F1, 8, D4F1);
    }
    for (int i = 0; i < 3; i++) {
        n += mk_std_line(buf + n, 0x4A2, 8, D4A2);
    }
    CHECK_MSG(n == 132, "six lines totalled %zu bytes, expected 132", n);

    feed(buf, 64, 40000);
    feed(buf + 64, n - 64, 80000);
    CHECK_MSG(sent_count == 6, "two-chunk burst sent %zu frames, want 6", sent_count);
    for (size_t i = 0; i < 3 && i < sent_count; i++) {
        check_frame(i, 0x4F1, D4F1, 8);
    }
    for (size_t i = 3; i < 6 && i < sent_count; i++) {
        check_frame(i, 0x4A2, D4A2, 8);
    }
}

// A leftover partial line must not be completed by the next command after a
// pause: the idle reset has to forget the abandoned frame, otherwise the next
// command's bytes finish it and the host sees a mangled copy of the old frame.
static void run_idle_after_partial(void) {
    reset();
    char buf[128];
    size_t n = 0;
    for (int i = 0; i < 2; i++) {
        n += mk_std_line(buf + n, 0x4F1, 8, D4F1);
    }
    size_t partial = mk_std_line(buf + n, 0x4F1, 8, D4F1);
    n += partial / 2;  // half a line, no terminator

    feed(buf, n, 40000);
    CHECK_MSG(sent_count == 2, "two complete lines + half sent %zu frames, want 2", sent_count);

    char line[32];
    size_t len = mk_std_line(line, 0x4A2, 8, D4A2);
    feed(line, len, 40000 + 200000);  // 200 ms later: past the 100 ms idle reset
    CHECK_MSG(sent_count == 3, "command after a pause sent %zu frames total, want 3", sent_count);
    if (sent_count >= 3) {
        check_frame(2, 0x4A2, D4A2, 8);
    }
}

// Same split line, but the caller's frame is poisoned between calls. can_tx_task
// declares its twai_message_t inside the loop, so the assembled frame must not
// depend on it.
static void run_poisoned_frame(void) {
    reset();
    char line[32];
    size_t len = mk_std_line(line, 0x4F1, 8, D4F1);
    for (size_t off = 0; off < len; ) {
        memset(&frame, 0xA5, sizeof(frame));
        size_t chunk = (off == 0) ? 9 : len - off;
        feed(line + off, chunk, 40000 + (int64_t)off * 1000);
        off += chunk;
    }
    CHECK_MSG(sent_count == 1, "poisoned-frame split line sent %zu frames, want 1", sent_count);
    if (sent_count >= 1) {
        check_frame(0, 0x4F1, D4F1, 8);
    }
}

// A DLC over 8 would walk past frame->data[8]; a zero DLC never satisfies the
// body loop's end test. Both come straight off the wire.
static void run_dlc_guards(void) {
    reset();
    feed("t4F19112233445566778\r", 21, 40000);  // dlc 9
    CHECK_MSG(sent_count == 0, "dlc=9 line sent %zu frames, want 0", sent_count);

    feed("t4F10\r", 6, 80000);  // dlc 0
    CHECK_MSG(sent_count == 1, "dlc=0 line sent %zu frames, want 1", sent_count);
    if (sent_count >= 1) {
        CHECK_MSG(sent[0].frame.data_length_code == 0,
                  "dlc=0 frame came out with dlc %u", sent[0].frame.data_length_code);
    }

    char line[32];
    size_t len = mk_std_line(line, 0x4A2, 8, D4A2);
    feed(line, len, 120000);
    CHECK_MSG(sent_count == 2, "line after a dlc=0 frame sent %zu frames total, want 2", sent_count);
    if (sent_count >= 2) {
        check_frame(1, 0x4A2, D4A2, 8);
    }
}

static const struct {
    const char *name;
    void (*fn)(void);
} suites[] = {
    {"slcan complete lines 40 ms apart", run_complete_lines},
    {"slcan one-call burst", run_burst_one_call},
    {"slcan recv-chunked burst", run_recv_chunked},
    {"slcan idle after partial", run_idle_after_partial},
    {"slcan poisoned frame", run_poisoned_frame},
    {"slcan dlc guards", run_dlc_guards},
};

#define NUM_SUITES (sizeof(suites) / sizeof(suites[0]))

int main(int argc, char **argv) {
    if (argc > 1) {
        for (size_t i = 0; i < NUM_SUITES; i++) {
            if (strstr(suites[i].name, argv[1]) != NULL) {
                suites[i].fn();
                return test_report(suites[i].name);
            }
        }
        fprintf(stderr, "no suite matching '%s'\n", argv[1]);
        return 1;
    }
    int rc = 0;
    for (size_t i = 0; i < NUM_SUITES; i++) {
        // fork per suite: the parser keeps its whole state in statics
        pid_t child = fork();
        if (child < 0) {
            perror("fork");
            return 1;
        }
        if (child == 0) {
            suites[i].fn();
            exit(test_report(suites[i].name));
        }
        int status = 0;
        waitpid(child, &status, 0);
        if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
            rc = 1;
        }
    }
    return rc;
}
