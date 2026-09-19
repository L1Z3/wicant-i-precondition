# MCP2518FD Classical CAN TWAI adapter

This component backs WiCAN `eb-fd` bus 1 with an MCP2518FD. The firmware still
uses Classical CAN frames with at most eight data bytes. `proto` continues to
use the existing MCP2515 driver.

## Source and local changes

The MIT-licensed Emandhal core is vendored in `vendor/` at revision
[`cb6c1eaad1147dcc19eaf02f7124c9ba20663a94`](https://github.com/Emandhal/MCP251XFD/tree/cb6c1eaad1147dcc19eaf02f7124c9ba20663a94)
(version 1.0.9). The upstream license is retained in `vendor/LICENSE.md`.
The import was verified against the Git blob hashes in the research checkout:

| File | Original Git blob SHA-1 |
| --- | --- |
| `MCP251XFD.c` | `5bcd8960d346a6b524255d9d3ca0c1045adcb4cf` |
| `MCP251XFD.h` | `1147d1c875b105841b76145554d00876f10209f3` |
| `ErrorsDef.h` | `321cc0dfebdce8908252496a2692d66ef35449eb` |
| `LICENSE.md` | `7c92976fefd4b66d3fada6c04f2c346284bcefcf` |

One local source fix returns an initialization SPI-read error before inspecting
the uninitialized read result. Upstream comments and formatting are otherwise
retained. `Conf_MCP251XFD.h` is WiCAN's project configuration. The adapter
validates its inputs; upstream's optional `CHECK_NULL_PARAM` is not enabled.

## Operation

- The GPIO ISR only notifies a worker. SPI transfers, event callbacks, and
  complete multi-transfer controller operations run under one task mutex.
- Startup uses 1 MHz SPI for reset/configuration, then the board's 10 MHz
  setting. SYSCLK is the supplied oscillator frequency with no PLL/divider.
  Configuration rejects SPI speeds above `0.85 * SYSCLK / 2`.
- Nominal timing supports WiCAN's 5 kbit/s–1 Mbit/s rates and configurable
  sample point. Bitrate must be exactly representable. The closest sample
  point is chosen, preferring finer time quanta on ties. At 40 MHz, WiCAN's
  87.5% setting is exact except 800 kbit/s, which uses 88%.
- FIFO1 is an eight-entry TX FIFO, FIFO2 is a 32-entry RX FIFO, and TEF has
  eight entries. Objects use 704 bytes of the 2 KB RAM. TXQ is disabled so
  lower CAN IDs cannot overtake earlier submissions within this node.
- Up to 32 accepted frames retain their original `twai_frame_t` pointers.
  TEF sequence numbers identify successful completions. Credits are released
  only after consuming TEF, so the number of outstanding events cannot exceed
  TEF capacity. Queue capacity includes hardware and software frames.
- One-shot mode permits one hardware frame at a time. Exhausted attempts
  reset that TX FIFO and complete the frame with `is_tx_success=false`;
  software-queued frames remain in order. Unlimited-retry mode pipelines eight.
- Standard/extended IDs, RTR, listen-only, and internal loopback are supported.
  Thirty-two independent single-ID mask filters are available. The first
  configured filter removes the default accept-all filter. FD-only, range,
  dual, and multi-ID list filters are not supported. FD receive objects are dropped.
- RX timestamps, when requested, are software timestamps in the requested
  resolution. Scheduled transmission and ISR transmit calls are not supported.

Callbacks run in **task context**, including cancellation callbacks during
disable. They may call `twai_node_receive_from_isr()` from `on_rx_done` despite
that API's name; other node API calls from callbacks are forbidden. WiCAN
selects the task or ISR FreeRTOS APIs according to the actual context.

Disable stops controller activity and returns all retained frame pointers
before returning. A cancellation does not prove the frame never reached the
wire: a frame can finish immediately before its TEF entry is processed.
SPI failures, corrupt/overflowed TEF, ECC/system faults, and bus-off stop the
node, fail outstanding frames, and notify the application to recreate it.
WiCAN's existing recovery task performs this recreation. Direct
`twai_node_recover()` is not implemented.

Enable logs the requested bitrate/oscillator/SPI settings and reads back
`CiCON`, `CiNBTCFG`, `OSC`, and `IOCON`. A service fault logs its reason, the
triggering `CiTREC`/`CiBDIAG1`, and configuration registers captured before
entering configuration mode. Error flags observed in earlier service passes
are accumulated until the next enable, so recovery does not hide an earlier
ACK or bit error. Validity flags distinguish failed reads from zero values.
These diagnostics do not measure the physical oscillator frequency.

Deletion requires a disabled node and must be serialized against application
API calls. It disables the interrupt source, waits for in-flight GPIO ISRs on
both cores, unregisters the handler, and asks the worker to exit. The worker is
joined before its mutexes, SPI device, and context are freed. WiCAN's
`node_lock` provides the application-side lifetime protection.

## Validation

Run `make -C test/mcp251xfd` from the repository root. This exercises the real
core and vendor SPI protocol against a register/RAM simulator, plus the actual
TWAI adapter against pthread-backed platform stubs. `make -C test/mcp251xfd
sanitize` adds AddressSanitizer and UndefinedBehaviorSanitizer. Sandboxes that
prevent LeakSanitizer process inspection may need `ASAN_OPTIONS=detect_leaks=0`;
the adapter tests also count platform resources explicitly.

These tests do not verify physical SPI timing, transceiver wiring, electrical
CAN behavior, ESP32 scheduling latency, or silicon errata. Follow the
[bench checklist](../../test/mcp251xfd/README.md) before relying on the
prototype on a vehicle network.
