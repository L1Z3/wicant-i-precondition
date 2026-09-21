# MCP2518FD Classical CAN TWAI adapter

This component backs WiCAN `eb-fd` bus 1 with an MCP2518FD. The firmware still
uses Classical CAN frames with at most eight data bytes. `proto` continues to
use the existing MCP2515 driver.

## EB-FD board controls

The supplied `eb-fd-start` firmware identifies bus 0 as SN65HVD233/U2 with
ESP32 GPIO 11 wired to RS, and bus 1 as TCAN3413/U4 with GPIO 12 wired to STB.
Both controls have external 10 kOhm pull-ups: high selects standby, low enables
normal operation. `main/hw_config.h` defines these only for EB-FD; proto retains
its original behavior. `main/can.c` controls the pins under each bus's lifecycle
lock, enabling the transceiver before its controller and restoring standby on
disable or failed enable. Bus 1 allows the TCAN3413's 30 us mode-change time
before proceeding ([datasheet, section 5.9](https://www.ti.com/lit/ds/symlink/tcan3413.pdf#page=9)).

MCP2518FD/U57 uses a 40 MHz crystal (X2), SPI2 SCLK 17/MOSI 16/MISO 15/CS 18,
and INT 7. GPIO 8 is unused; the controller's GPIO0/GPIO1 remain inputs.
The supplied firmware confirms the LED and VBAT mappings already used by
WiCAN. The ESP32 module's flash/PSRAM capacity remains provisional.

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
- Transmit callers put copied frames and original completion tokens into a
  submission ring under a separate short mutex, without waiting for the SPI
  service mutex. The worker admits those frames into the portable core. One
  credit limit covers the submission ring, core queue, and hardware together;
  idle status and cancellation include all accepted frames. A disable/enable
  generation check rejects senders that waited across that transition.
- Startup uses 1 MHz SPI for reset/configuration, then the board's 10 MHz
  setting. SYSCLK is the supplied oscillator frequency with no PLL/divider.
  Configuration rejects SPI speeds above `0.85 * SYSCLK / 2`.
- EB-FD uses polling SPI without DMA. Transfers are at most 18 bytes, fitting
  the peripheral's CPU FIFO and avoiding DMA temporary-buffer allocation and
  copying for short register accesses. Vendor transfers larger than this are
  split into 16-byte data chunks.
- WiCAN sets `exclusive_spi` because MCP2518FD is the only device on SPI2.
  This reserves the bus for the node's lifetime, avoiding per-transfer bus
  arbitration. The reservation is released before speed changes or deletion,
  including failed creation. Other callers can leave this flag clear when
  sharing a bus; the driver mutex still serializes access to this device.
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
- On EB-FD, WiCAN's shared RX/forward task runs unpinned at priority 21, above
  the MCP worker (20) and the pinned IDF version's WPA3 AP authentication task
  (19). It can run on either CPU as work becomes available. TX submission uses
  the separate short mutex, so forwarding need not wait for a SPI service pass.
  When draining a persistent backlog, it checks elapsed time between complete
  frames and blocks for one tick after 8 ms. It holds neither the state-machine
  nor TX locks during that pause. This budget includes time spent blocked or
  preempted, and does not bound individual frame handling. Proto/v300 retain
  unpinned priority 7 without this extra pause.
- The SPI worker runs on core 1 at priority 20 on dual-core targets. The worker
  checks elapsed busy time between service passes and blocks for one tick
  after 8 ms without reaching its notification wait. Both mutexes are released
  before that pause, and pending notifications are preserved. One tick is
  1 ms in the current firmware. This gives lower-priority work an opportunity
  to run under continuous notifications; it does not bound an individual SPI
  transfer or callback. The watchdog remains enabled. The initial 8 ms budget
  needs throughput validation on hardware.
- The service loop uses interrupt flags to skip inactive RX/TEF FIFOs. It
  combines adjacent FIFO status/address reads and error-counter/diagnostic
  reads into bursts, while retaining the per-pass bus-off checks. It refills
  TX before draining an RX burst so reception cannot delay restarting TX.
  The first TX refill reuses the status/address snapshot from that pass's
  error checks; subsequent UINC operations and one-shot resets require fresh
  snapshots. After all loaded frames complete, TEF draining stops without an
  extra empty check. Any unexpected remaining event asserts INT for the next
  pass, where ownership and sequence validation still apply.
  Events arriving after the interrupt snapshot leave INT asserted and are
  serviced on the next pass.
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

Throughput instrumentation is **compiled out by default**. To restore it,
enable `CONFIG_MCP251XFD_PERF_DIAGNOSTICS` under `MCP2518FD driver` in
`menuconfig`. This includes both the driver's service statistics and WiCAN's
software RX queue timestamps; disabling it also removes their counters,
timestamp calls and 2 KiB of extra queue storage. The scheduling budgets and
ordinary queue-drop/controller-fault warnings remain active.

With diagnostics enabled, a WARN-level `service window` line appears at most
once per second when busy intervals reach the budget or long gaps/passes
occur. It accumulates worst times over the whole reporting window instead of
sampling one busy slice:

- `TX` counts successful TEF completions; `RX` counts frames handed to the
  application callback, which may subsequently reject them if its queue is full.
- `passes` and `pauses` count service passes and scheduled one-tick pauses.
- `max gap/pass/busy` gives microseconds between service starts, within a
  service pass, and within a busy interval. Gap includes normal idle polling
  (up to approximately 10 ms), scheduling, mutex waits and diagnostic logging;
  pass/busy times also include preemption. These are not pure SPI timings.
- `INT` is the final pass's `CiINT` snapshot. `0010` is TEF activity; `0012`
  adds RX activity. These flags alone do not indicate a controller error.
- `HW RX overruns` counts cumulative observed FIFO overflow flags, not exact
  lost frames. It is separate from `can_receive: rx queue full` counters.

With that option enabled, EB-FD also timestamps entries in the shared
application RX queue. An `RX scheduling` warning reports the window's worst
queue residence time and gap between dequeues when residence exceeds 20 ms,
at most once per second.
The reported gap can include idle time, processing, scheduling, mutex/TX-slot
waits and logging; queue age indicates an actual waiting frame. `queued` is a
current queue-depth snapshot, not its high-water mark. The timestamp adds
eight bytes per queue entry (2 KiB for 256 entries); frame capacity is unchanged.

Deletion requires a disabled node and must be serialized against application
API calls. It disables the interrupt source, waits for in-flight GPIO ISRs on
both cores, unregisters the handler, and asks the worker to exit. The worker is
joined before its mutexes, SPI device, and context are freed. WiCAN's
`node_lock` provides the application-side lifetime protection.

After joining the worker, deletion requests MCP2518FD low-power mode (LPM),
with controller interrupts and CAN wake disabled. It performs no SPI readback
after this request: asserting CS would wake the chip again. CS stays held high
across SPI-device removal and ESP32 light sleep. Creation configures CS before
releasing the hold, wakes the controller at 1 MHz, waits for OSCRDY, and restores
the complete configuration and RAM. LPM discards both on wake-up; see the
[MCP2518FD datasheet, OSC register](https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/External-CAN-FD-Controller-with-SPI-Interface-DS20006027B.pdf#page=16).

This also covers WiCAN's existing sleep path, which disables and deletes both
CAN nodes before ESP32 light sleep. Recovery and configuration changes recreate
the controller through the same path. A low-power request that fails is logged;
resource cleanup still completes so an unreachable controller cannot leak a
node. `twai_node_disable()` alone retains configuration mode and supports
re-enabling the same node; LPM is requested only when disposing of that node.

## Validation

The host SPI model counts 3 transactions for an idle service pass, 5 for one
TX submission, and 12 for one TX submission plus RX and completion. The same
workloads after the first throughput pass used 3, 6 and 14, and before it used
6, 10 and 18 respectively. TX completion plus refill uses 8, previously 10.
These counts are regression checks, not measured board throughput; they also
exclude the CPU savings from avoiding DMA and repeated SPI bus arbitration.

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

The latest hardware retest reports no drops while joining Wi-Fi or using the
HTTP interface, with a small remaining burst when connecting SavvyCAN (76
bus-1 TX-pool drops in the supplied log). See the
[findings](../../test/mcp251xfd/bringup-findings.md) for test history and limits.
