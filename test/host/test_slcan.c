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
static bool bridge_locked;  // the MITM-lock answer; a suite may flip it

// What the parser asked the bus to do. Recording rather than ignoring these
// keeps the suite honest: a command that answers OK must also reach can.c.
static uint8_t silent_state;
static uint8_t loopback_state;
static uint8_t rate_value;
static size_t rate_calls;
static uint32_t last_filter;
static size_t filter_calls;
static uint32_t last_mask;
static size_t mask_calls;

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
void can_set_silent(can_bus_t bus, uint8_t flag) { (void)bus; silent_state = flag; }
void can_set_loopback(can_bus_t bus, uint8_t flag) { (void)bus; loopback_state = flag; }
void can_set_bitrate(can_bus_t bus, uint8_t rate) { (void)bus; rate_value = rate; rate_calls++; }
void can_set_filter(can_bus_t bus, uint32_t f) { (void)bus; last_filter = f; filter_calls++; }
void can_set_mask(can_bus_t bus, uint32_t m) { (void)bus; last_mask = m; mask_calls++; }

// MITM forwarding holds bus 0's configuration; while it does, the parser must
// answer a refused command with BEL. The suites below flip this to check that.
bool can_bridge_locked(void) { return bridge_locked; }

// Bus 0's current rate. Only read under the lock, by the 'S' refusal check.
uint8_t can_get_bitrate(can_bus_t bus) { (void)bus; return CAN_500K; }

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

// Extended-ID equivalent: 'T'/'R' <8 hex id> <dlc hex> [data hex] '\r'.
static size_t mk_ext_line(char *out, uint32_t id, uint8_t dlc, const uint8_t *data, int rtr) {
    static const char hex[] = "0123456789ABCDEF";
    size_t n = 0;
    out[n++] = rtr ? 'R' : 'T';
    for (int shift = 28; shift >= 0; shift -= 4) {
        out[n++] = hex[(id >> shift) & 0xF];
    }
    out[n++] = hex[dlc & 0xF];
    if (!rtr) {
        for (uint8_t b = 0; b < dlc; b++) {
            out[n++] = hex[(data[b] >> 4) & 0xF];
            out[n++] = hex[data[b] & 0xF];
        }
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

static void check_reply(const char *want, size_t want_len, const char *what) {
    CHECK_MSG(response_count == want_len && memcmp(response, want, want_len) == 0,
              "%s answered %zu bytes, want %zu", what, response_count, want_len);
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
    bridge_locked = false;
    silent_state = 0;
    loopback_state = 0;
    rate_value = 0;
    rate_calls = 0;
    last_filter = 0;
    filter_calls = 0;
    last_mask = 0;
    mask_calls = 0;
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

// MITM forwarding holds bus 0's configuration: a command that would change it
// is answered with BEL, not with the OK that would be a lie. Once the lock is
// released the same command is accepted again.
static void run_locked_refusals(void) {
    reset();
    bridge_locked = true;

    feed("C\r", 2, 40000);  // close bus 0
    CHECK_MSG(response_count == 1 && response[0] == '\a',
              "close under MITM lock answered %zu bytes starting 0x%02x, want BEL",
              response_count, response_count ? (unsigned char)response[0] : 0);

    response_count = 0;
    feed("S0\r", 3, 80000);  // change bus 0's bitrate
    CHECK_MSG(response_count == 1 && response[0] == '\a',
              "bitrate change under MITM lock answered %zu bytes starting 0x%02x, want BEL",
              response_count, response_count ? (unsigned char)response[0] : 0);

    response_count = 0;
    feed("Y\r", 2, 120000);  // open in loopback
    CHECK_MSG(response_count == 1 && response[0] == '\a',
              "loopback open under MITM lock answered %zu bytes starting 0x%02x, want BEL",
              response_count, response_count ? (unsigned char)response[0] : 0);

    bridge_locked = false;
    response_count = 0;
    feed("C\r", 2, 160000);  // close again, now allowed
    CHECK_MSG(response_count == 1 && response[0] == '\r',
              "close with the lock released answered %zu bytes starting 0x%02x, want CR",
              response_count, response_count ? (unsigned char)response[0] : 0);
}

// Extended-ID frames carry the 29-bit ID in eight hex digits after 'T'. The
// assembler is shared with the 11-bit path, so this pins that the word and the
// extd flag come out right, including when the line is split.
static void run_extended_ids(void) {
    reset();
    static const uint8_t data[8] = {0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88};
    char line[40];
    size_t len = mk_ext_line(line, 0x1ABCDEF0u, 8, data, 0);

    feed(line, len, 40000);
    CHECK_MSG(sent_count == 1, "extended line sent %zu frames", sent_count);
    if (sent_count == 1) {
        CHECK_MSG(sent[0].frame.identifier == 0x1ABCDEF0u,
                  "extended id 0x%x, want 0x1ABCDEF0", (unsigned)sent[0].frame.identifier);
        CHECK_MSG(sent[0].frame.extd == 1, "extended line came out as std");
        CHECK_MSG(sent[0].frame.rtr == 0, "extended line came out as RTR");
        CHECK_MSG(sent[0].frame.data_length_code == 8,
                  "extended dlc %u", sent[0].frame.data_length_code);
        CHECK_MSG(memcmp(sent[0].frame.data, data, 8) == 0, "extended data mismatch");
    }

    // The same line split across two calls, as a 64-byte recv hands it over.
    sent_count = 0;
    feed(line, 5, 80000);
    feed(line + 5, len - 5, 85000);
    CHECK_MSG(sent_count == 1, "split extended line sent %zu frames", sent_count);
    if (sent_count == 1) {
        CHECK_MSG(sent[0].frame.identifier == 0x1ABCDEF0u,
                  "split extended id 0x%x", (unsigned)sent[0].frame.identifier);
        CHECK_MSG(sent[0].frame.extd == 1, "split extended line came out as std");
        CHECK_MSG(sent[0].frame.data_length_code == 8,
                  "split extended dlc %u", sent[0].frame.data_length_code);
    }
}

// 'r'/'R' are RTR: the identifier and DLC travel, the payload does not. A data
// frame after one must not inherit the rtr flag.
static void run_rtr_frames(void) {
    reset();
    feed("r4F18\r", 6, 40000);
    CHECK_MSG(sent_count == 1, "std RTR line sent %zu frames", sent_count);
    if (sent_count == 1) {
        CHECK_MSG(sent[0].frame.identifier == 0x4F1,
                  "RTR id 0x%x", (unsigned)sent[0].frame.identifier);
        CHECK_MSG(sent[0].frame.extd == 0, "std RTR came out extd");
        CHECK_MSG(sent[0].frame.rtr == 1, "std RTR came out as data");
        CHECK_MSG(sent[0].frame.data_length_code == 8,
                  "RTR dlc %u", sent[0].frame.data_length_code);
    }

    sent_count = 0;
    feed("R1ABCDEF02\r", 11, 80000);
    CHECK_MSG(sent_count == 1, "ext RTR line sent %zu frames", sent_count);
    if (sent_count == 1) {
        CHECK_MSG(sent[0].frame.identifier == 0x1ABCDEF0u,
                  "ext RTR id 0x%x", (unsigned)sent[0].frame.identifier);
        CHECK_MSG(sent[0].frame.extd == 1, "ext RTR came out std");
        CHECK_MSG(sent[0].frame.rtr == 1, "ext RTR came out as data");
        CHECK_MSG(sent[0].frame.data_length_code == 2,
                  "ext RTR dlc %u", sent[0].frame.data_length_code);
    }

    sent_count = 0;
    char line[32];
    size_t len = mk_std_line(line, 0x4A2, 8, D4A2);
    feed(line, len, 120000);
    CHECK_MSG(sent_count == 1 && sent[0].frame.rtr == 0,
              "data line after RTR came out rtr=%d", sent_count ? sent[0].frame.rtr : -1);
}

// Loopback is a parser-side setting: 'Y' turns it on and the frame the host
// sees its own echoes through carries self=1; 'O' clears it again.
static void run_loopback(void) {
    reset();
    feed("Y\r", 2, 40000);
    CHECK_MSG(loopback_state == 1 && silent_state == 0,
              "Y left loopback=%u silent=%u", loopback_state, silent_state);

    char line[32];
    size_t len = mk_std_line(line, 0x4F1, 8, D4F1);
    feed(line, len, 80000);
    CHECK_MSG(sent_count == 1, "frame after Y sent %zu", sent_count);
    if (sent_count == 1) {
        CHECK_MSG(sent[0].frame.self == 1,
                  "loopback frame self=%u, want 1", sent[0].frame.self);
    }

    sent_count = 0;
    feed("O\r", 2, 120000);
    CHECK_MSG(loopback_state == 0, "O left loopback=%u", loopback_state);
    feed(line, len, 160000);
    CHECK_MSG(sent_count == 1, "frame after O sent %zu", sent_count);
    if (sent_count == 1) {
        CHECK_MSG(sent[0].frame.self == 0,
                  "non-loopback frame self=%u, want 0", sent[0].frame.self);
    }
}

// 'M'/'m' are the accept code and mask: the hex digits arrive most significant
// first and come out as one 32-bit word. While MITM holds the bus only the
// accept-all values may pass.
static void run_accept_code_and_mask(void) {
    reset();
    feed("M12345678\r", 10, 40000);
    CHECK_MSG(filter_calls == 1 && last_filter == 0x12345678u,
              "M12345678 set filter 0x%08x (%zu calls)", (unsigned)last_filter, filter_calls);

    feed("mFFFFFFFF\r", 10, 80000);
    CHECK_MSG(mask_calls == 1 && last_mask == 0xFFFFFFFFu,
              "mFFFFFFFF set mask 0x%08x (%zu calls)", (unsigned)last_mask, mask_calls);

    bridge_locked = true;
    response_count = 0;
    feed("M00000001\r", 10, 120000);
    CHECK_MSG(response_count == 1 && response[0] == '\a',
              "M under lock answered %zu bytes, want BEL", response_count);

    response_count = 0;
    feed("M00000000\r", 10, 160000);
    CHECK_MSG(response_count == 1 && response[0] == '\r',
              "accept-all M under lock answered %zu bytes, want OK", response_count);

    response_count = 0;
    feed("mFFFFFFFE\r", 10, 200000);
    CHECK_MSG(response_count == 1 && response[0] == '\a',
              "m under lock answered %zu bytes, want BEL", response_count);

    response_count = 0;
    feed("mFFFFFFFF\r", 10, 240000);
    CHECK_MSG(response_count == 1 && response[0] == '\r',
              "accept-all m under lock answered %zu bytes, want OK", response_count);
}

// The text replies are host-visible strings; 'Z1'/'Z0' add or drop the four
// hex digits of timestamp in slcan_parse_frame()'s output.
static void run_text_replies(void) {
    reset();
    response_count = 0;
    feed("V\r", 2, 40000);
    check_reply("V2011\r", 6, "V");

    response_count = 0;
    feed("N\r", 2, 80000);
    check_reply("N43010123\r", 10, "N");

    response_count = 0;
    feed("F\r", 2, 120000);
    check_reply("F00\r", 4, "F");

    feed("Z1\r", 3, 160000);
    twai_message_t f = {0};
    f.identifier = 0x4F1;
    f.data_length_code = 1;
    f.data[0] = 0xAA;
    char buf[64];
    int8_t n = slcan_parse_frame((uint8_t *)buf, &f);
    CHECK_MSG(n == 12, "timestamped frame encoded %d bytes, want 12", (int)n);
    for (int i = n - 4; i < n - 1; i++) {
        bool hex = (buf[i] >= '0' && buf[i] <= '9') || (buf[i] >= 'A' && buf[i] <= 'F');
        CHECK_MSG(hex, "timestamp digit %d is 0x%02x", i, (unsigned char)buf[i]);
    }

    feed("Z0\r", 3, 200000);
    n = slcan_parse_frame((uint8_t *)buf, &f);
    CHECK_MSG(n == 8, "frame without timestamp encoded %d bytes, want 8", (int)n);
}

// 'S' with an index selects from sl_bitrate[]; an index past the table is
// ignored rather than indexing off the end.
static void run_bitrate_table(void) {
    reset();
    for (int i = 0; i <= 8; i++) {
        char cmd[8];
        int n = snprintf(cmd, sizeof(cmd), "S%d\r", i);
        size_t before = rate_calls;
        feed(cmd, (size_t)n, 40000 + (int64_t)i * 1000);
        CHECK_MSG(rate_calls == before + 1 && rate_value == sl_bitrate[i],
                  "S%d set rate %u (%zu calls), want %u", i, rate_value, rate_calls, sl_bitrate[i]);
    }
    size_t before = rate_calls;
    feed("S9\r", 3, 60000);
    CHECK_MSG(rate_calls == before, "S9 changed the rate (%zu calls)", rate_calls);
}

// A host can stream without ever terminating a line. The parser must not
// accumulate those bytes in its scratch buffer and must still take the next
// real command.
static void run_buffer_flood(void) {
    reset();
    char chunk[256];
    memset(chunk, 'x', 255);  // unknown command bytes are skipped in SL_HEADER
    for (int i = 0; i < 10; i++) {
        feed(chunk, 255, 40000 + (int64_t)i * 1000);
    }
    CHECK_MSG(sent_count == 0, "flood of 2550 bytes sent %zu frames", sent_count);

    char line[32];
    size_t len = mk_std_line(line, 0x4F1, 8, D4F1);
    feed(line, len, 60000);
    CHECK_MSG(sent_count == 1, "line after the flood sent %zu frames, want 1", sent_count);
    if (sent_count == 1) {
        check_frame(0, 0x4F1, D4F1, 8);
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
    {"slcan locked refusals", run_locked_refusals},
    {"slcan extended ids", run_extended_ids},
    {"slcan rtr frames", run_rtr_frames},
    {"slcan loopback", run_loopback},
    {"slcan accept code and mask", run_accept_code_and_mask},
    {"slcan text replies", run_text_replies},
    {"slcan bitrate table", run_bitrate_table},
    {"slcan buffer flood", run_buffer_flood},
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
