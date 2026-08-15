# Logging Overflow Audit

Security review of buffer overflows reachable through the logging paths of this
firmware. Covers application code (`main/`), the local components
(`components/`), and the vendored ESP-IDF log subsystem
(`esp-idf/components/log/`).

## Summary

| Area | Finding | Severity | Status |
|------|---------|----------|--------|
| `main/config_server.c` | WebSocket frame `memcpy` into 65-byte buffer, then logged via `ESP_LOG_BUFFER_HEXDUMP` | High | Fixed |
| `main/config_server.c` | `%s` log of a non-NUL-terminated file buffer | Medium | Fixed |
| `main/autopid.c` | `%s` log / parse over-read after unbounded `data_start += 9`; `header_str[9]` `strncpy` | Medium (latent) | Fixed |
| `main/main.c` | Log/parse consumers trust `usLen` | High (defense-in-depth) | Fixed |
| `components/*` | `ESP_LOG*` overflows | — | None found |
| `esp-idf/components/log/include/` | Buffer overflow | — | None (declarations only) |
| `esp-idf/components/log/src/buffer/log_buffers.c` | `memcpy` length rounded up to /4 → OOB read | Low | Open (vendored submodule) |
| `esp-idf/components/log/src/log_format_binary.c` | `int16_t` length truncation in binary log | Low | Open (vendored submodule) |

---

## 1. Application code (`main/`)

### 1.1 WebSocket frame overflow (High, fixed)

`config_server.c` `ws_handler` (`/ws`, registered at line ~1630, unauthenticated)
copied a client-controlled-length WebSocket payload into a fixed 65-byte queue
buffer:

```c
static xdev_buffer rx_buffer;
memcpy(rx_buffer.ucElement, ws_pkt.payload, ws_pkt.len);   // ws_pkt.len client-controlled
rx_buffer.usLen = ws_pkt.len;
```

`xdev_buffer.ucElement` is `uint8_t[DEV_BUFFER_LENGTH]` = 65 bytes
(`types.h:25`). `ws_pkt.len` comes straight from the frame header (the handler
`calloc(1, ws_pkt.len + 1)`s it with no upper bound). Any WebSocket client could
send a frame > 65 bytes and overflow the static buffer (`.bss` write overflow).

The logging connection: the queued item is consumed by `can_tx_task`
(`main.c`), which immediately did

```c
ESP_LOG_BUFFER_HEXDUMP(TAG, ucTCP_RX_Buffer.ucElement, ucTCP_RX_Buffer.usLen, ESP_LOG_INFO);
```

with `usLen` equal to the untruncated attacker length — an out-of-bounds read
inside the log call, followed by the same bad length flowing into
`slcan_parse_str()`.

**Fix:** receive the full frame, then clamp only the copy:

```c
size_t copy_len = ws_pkt.len;
if (copy_len > DEV_BUFFER_LENGTH) {
    copy_len = DEV_BUFFER_LENGTH;
    ESP_LOGW(TAG, "websocket frame too large: %d bytes, truncating to %d", ...);
}
memcpy(rx_buffer.ucElement, ws_pkt.payload, copy_len);
rx_buffer.usLen = (int)copy_len;
```

Notes:
- Clamping `ws_pkt.len` *before* the second `httpd_ws_recv_frame` is **not**
  correct: `ws_pkt.len` is reused as `max_len`, so shrinking it leaves the rest
  of the frame unread and desynchronizes the WebSocket stream. Truncation must
  be applied to the copy length after the frame is fully received.
- The `ws_pkt.len == 0` early return is required to avoid
  `memcpy(dst, NULL, 0)` UB and an empty queue item.
- The TCP and UDP RX paths in `comm_server.c` already cap reads at
  `sizeof(ucElement) - 1`; the WebSocket path was the outlier.

### 1.2 Unterminated `%s` log (Medium, fixed)

`config_server.c` `load_pid_auto_config_handler`:

```c
char *buf = (char *)malloc(file_size + 1);
size_t read_len = fread(buf, 1, file_size, fd);
...
char *last_brace = strrchr(buf, '}');              // over-reads if no '}'
...
ESP_LOGI(TAG, "Sending response: %s", buf);        // over-reads if no '}'
```

`buf[file_size]` was never written; the buffer was only terminated when the file
contained a `}`. `car_data.json` is written by the unauthenticated
`/upload/car_data.json` handler with arbitrary content, so a crafted upload made
both `strrchr` and the `%s` log read out of bounds.

**Fix:** `buf[file_size] = '\0';` immediately after the `fread` success check.

### 1.3 `parse_elm327_response` over-reads (Medium, latent; fixed)

`autopid.c` `parse_elm327_response` had two issues feeding `%s` log calls:

1. `data_start += 9` in the `case 2` branch could push `data_start` past the
   string's NUL for short frames, then
   `ESP_LOGV/W("... %s", data_start)` and the `while (*data_start != '\0')`
   loop read out of bounds.
2. `strncpy(header_str, frame, header_length)` ran before `header_length` was
   validated; a first token > 8 chars overflows the stack buffer `header_str[9]`.

Reachability is low today (autopid forces `ath1` and filters `ath0`, so the
2-char-header path is effectively dead), but the code is fragile.

**Fix:** validate `header_length` (`<= 0 || > 8` → skip frame) before the
`strncpy`, and guard `case 2` with `strlen(data_start) >= 9`.

### 1.4 `can_tx_task` trusts `usLen` (High defense-in-depth, fixed)

`main.c` `can_tx_task` consumed `ucTCP_RX_Buffer.usLen` (an `int`) for the
hexdump log and `slcan_parse_str` without bounding it to the 65-byte buffer.

**Fix:** clamp `usLen` to `[0, DEV_BUFFER_LENGTH]` after the queue receive. This
is defense-in-depth behind fix 1.1, but protects against any future producer
that enqueues a corrupted length.

---

## 2. Local components (`components/`)

Audited all four local components for `ESP_LOG*` usage. **No overflows found.**

- `ha_webhooks/` — all string copies use `strlcpy` (guaranteed NUL-termination)
  or the struct is `memset(0)`; `%s` args are cJSON `valuestring`,
  `req->uri`, literals, or `esp_err_to_name()`. Request body is capped at 2048
  and NUL-terminated before logging.
- `filesystem/` — `%s` args are `entry->d_name` (NUL-terminated dirent),
  caller-supplied paths, and literals.
- `esp_twai_mcp2515/` — no logging calls at all.
- `debug_logs/` — uses its own `DEBUG_LOG*` macros (not `ESP_LOG*`);
  `debug_logs_vlog`/`debug_logs_send_line` are bounds-clamped
  (`vsnprintf`/`snprintf` with clamped lengths, `strnlen` capped at
  `DEBUG_LOG_MAX_LINE - 1`).

---

## 3. Vendored ESP-IDF log subsystem (`esp-idf/components/log/`)

The fork carries no local modifications to `components/log` (verified via
`git status`/`git log`); this is stock ESP-IDF v5.5.3.

### 3.1 `include/` — no overflows

All 11 public headers plus the 8 `esp_private/` headers were reviewed. The
directory contains only macros, `static inline` passthroughs, and function
declarations — no runtime buffers. The only arrays are compile-time-sized string
literals (`esp_log_args.h`, `esp_private/log_attr.h`), sized by their
initializers.

The overflow-prone code those headers expose lives in `src/`:

### 3.2 `src/buffer/log_buffers.c` — OOB read (Low, open)

`print_buffer()`, the implementation behind
`ESP_LOG_BUFFER_HEX/CHAR/HEXDUMP` (declared in `include/esp_log_buffer.h`):

```c
char temp_buffer[BYTES_PER_LINE + 3];                  // 19 bytes
...
int bytes_cur_line = MIN(BYTES_PER_LINE, buff_len);    // 1..16
if (!esp_ptr_byte_accessible(buffer)) {
    memcpy(temp_buffer, buffer, (bytes_cur_line + 3) / 4 * 4);  // rounds UP to /4
    ptr_line = temp_buffer;
}
```

The copy length is rounded **up** to a multiple of 4. When the remaining
`buff_len` is not a multiple of 4 (e.g. 17 → second line copies 4 bytes for 1
valid byte), it reads 1–3 bytes past the end of the caller's buffer for
non-byte-accessible memory (flash/ROM). The write side is safe (max copy 16 ≤
19), so this is a source over-read.

Suggested fix: `memcpy(temp_buffer, buffer, MIN(bytes_cur_line, (int)sizeof(temp_buffer)));`
(or keep the word-copy but clamp the source to the remaining valid length).

### 3.3 `src/log_format_binary.c` — `int16_t` truncation (Low, open)

```c
int len = (pkg_info->buffer_len) ? pkg_info->buffer_len : strlen(ptr);
int16_t pkg_str_len = 1 - len;                        // overflows for len > 32768
...
for (unsigned i = 0; i < MAX(len, 2); i++) {          // reads len bytes from ptr
    pkg_len += output(&ptr[i], sizeof(uint8_t), pkg_info);
}
```

`buffer_len` originates from the 16-bit `buff_len` (up to 65535), so `1 - len`
can overflow `int16_t`, corrupting the encoded string length in the binary log
stream. The loop then reads `len` bytes from the pointer (an over-read when
`buff_len` exceeds the real buffer — a caller-contract issue, but the `int16_t`
wrap is IDF's own).

Suggested fix: widen `pkg_str_len` to `int32_t` and/or clamp `len` to the
binary-log package size limit.

### 3.4 Checked, not overflows

- `esp_log_timestamp_str()` writes to a caller buffer with **no size argument**
  (`include/esp_private/log_timestamp.h`); its only caller
  (`src/log_format_text.c:51`) provides 32 bytes, sufficient for the 22-byte
  worst case (`CONFIG_LOG_TIMESTAMP_SOURCE_SYSTEM_FULL`).
- Hex/char/hexdump line buffers (`3*16`, `16+1`, and the 82-byte hexdump
  buffer) are sized exactly to what the line writers emit.
- `esp_log_system_timestamp()`'s manual digit loop and the `esp_log_args.h`
  arg-type packing (48-arg limit) are bounded.

---

## Fix status

- **Fixed** (this fork, uncommitted): `main/config_server.c`,
  `main/autopid.c`, `main/main.c` (see sections 1.1–1.4), plus
  `main/wc_uart.c` (dead-code `uart_read_bytes` capped to `sizeof(ucElement)`).
- **Open**: `esp-idf/components/log/src/buffer/log_buffers.c` and
  `esp-idf/components/log/src/log_format_binary.c` are upstream submodule code;
  no changes made pending a decision on whether to carry additional local
  patches in the vendored `esp-idf/`.
