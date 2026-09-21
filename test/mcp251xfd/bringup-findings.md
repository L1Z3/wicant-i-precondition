# EB-FD throughput findings

## First optimization pass, 2026-09-21

Tested on the EB-FD prototype with a 40 MHz MCP2518FD oscillator, Classical
CAN at the usual 500 kbit/s on both buses, and MITM forwarding on the user's
M-CAN bus. The starting firmware was `efa9a71`, which added the board's
transceiver standby controls and MCP2518FD low-power lifecycle support.

Before optimization, forwarding worked but the shared software RX queue
overflowed continuously: approximately 1,100 frames/s from bus 0 and 150
frames/s from bus 1. These `can_receive: rx queue full` counters count frames
rejected by the shared application queue. They do not identify hardware RX
FIFO overflow or CAN bus errors.

The first optimization pass made the following changes:

- Reserve the dedicated SPI bus for the node's lifetime, as the MCP2515
  driver already does, avoiding arbitration on every short transfer.
- Use polling SPI without DMA and cap transfers at 18 bytes. This avoids
  temporary DMA buffer allocation and copying for small, unaligned transfers;
  larger vendor operations are split into 16-byte data chunks.
- Read adjacent FIFO status/address and diagnostic registers in bursts.
- Use interrupt flags to skip inactive RX and completion FIFOs, and refill
  TX before draining RX bursts.
- Skip timestamp acquisition when the caller has not requested timestamps.

Queue sizes, SPI clock, CAN timing, CPU frequency and task priorities were
unchanged. The simulator measured the following SPI transaction counts:

| Service workload | Before | After |
| --- | ---: | ---: |
| Idle | 6 | 3 |
| One TX submission | 10 | 6 |
| One TX submission, RX frame and TX completion | 18 | 14 |

The user subsequently reported **all observed drops eliminated in normal
use**. This establishes a substantial improvement for that workload, but is
not a measured lossless-throughput limit. Test duration and independent
end-to-end frame counts were not supplied, and the individual changes were
not tested separately, so their individual contributions are unknown.

Validation before that hardware test: core and adapter host tests, including
burst traffic, FIFO wrap, late interrupt arrival and fault/lifecycle cleanup;
ASan/UBSan; and successful `v300`, `proto` and `eb-fd` firmware builds.

## Correction: the AP connection failure was a password mismatch

The user initially reported that attempts to join the AP timed out, with
intermittent CAN RX queue drops during the attempts. The failure also
occurred with CAN traffic disconnected, producing `wifi:rm mis` warnings.
The user subsequently identified a changed/reset Wi-Fi password as the
cause of the connection failure. The source of the password change was not
established. These observations do not demonstrate an AP authentication
regression caused by the MCP2518FD optimizations.

A temporary diagnostic image restored Wi-Fi INFO logs otherwise hidden by
`app_main()`'s global WARN filter. That logging override is no longer active;
the connection failure no longer needs that investigation. Numeric disconnect
reason logging remains available when the Wi-Fi application tag is at INFO.

## Remaining issue: CAN drops during Wi-Fi activity

With the password corrected, the user still observed CAN frame drops while
Wi-Fi connects, serves a page, or streams SavvyCAN traffic. Prototype 1
handles this workload without reported drops. Normal-use forwarding without
that Wi-Fi activity remains free of observed drops after the first pass.
No updated loss rate or independent wire-level frame count was supplied.

Source review identified two places where the receive/forward path could be
delayed:

- The MCP2518FD adapter took the core/SPI mutex twice for every transmit,
  even with timeout zero. That mutex covers a complete controller service
  pass, including RX bursts and callbacks. A sender could therefore wait for
  SPI servicing before returning to the shared RX queue. In comparison, the
  MCP2515 driver queues frames directly when its transmitter is already busy.
- The MCP worker runs on core 1 at priority 20, but the shared receive/forward
  task was unpinned at priority 7. Wi-Fi/lwIP use core 0, while network
  application work and WPA3 authentication can also compete for core 1.

The second pass added a TX submission ring protected by a separate short
mutex. The worker moves submissions into the portable core and remains the
owner of SPI servicing. The existing TX credit limit still covers **all**
accepted frames, including the submission ring, core queue and hardware;
there are still at most 32 outstanding frames. Original completion pointers,
FIFO order, queue backpressure, and cancellation on disable/fault are
preserved. Senders waiting across a disable/enable transition are rejected
using a generation check. Idle status includes staged submissions.

For EB-FD on dual-core targets, that pass also put the receive/forward task on core 1
at priority 19: below the MCP worker (20), above network application tasks,
and sharing priority with WPA3 authentication if it runs there. It blocked on
RX while idle and retained the existing bounded TX backpressure. Proto and
v300 retained their previous task settings. RX queue size, total TX capacity,
SPI clock, CAN timing and CPU frequency were unchanged.

A host regression test pauses the worker while it owns the core mutex and
attempts zero-timeout submissions from another thread. It fails on the
first-pass driver and passes after the submission change. Further cases
verify the shared 32-frame limit, pending-work idle status, ordered successful
completion, and exactly-once cancellation of staged frames on disable or SPI
failure. These tests establish removal of that SPI-lock dependency; they do
not measure ESP32 scheduling or prove that Wi-Fi-related losses are fixed.

The second-pass core/adapter tests and ASan/UBSan checks passed, and all three
firmware variants built successfully. That image was copied to
`build.eb-fd/wican-eb-fd-wifi-load.bin`. Its subsequent hardware test failed.

## Second-pass regression observed on hardware

The user reported persistent drops even before joining Wi-Fi. Bus 0's shared
RX queue counter increased from 507 at 1.742 s to 6,221 at 23.747 s, roughly
260 drops/s. Bus 1 also dropped frames, at a variable rate. After connecting,
bus 0 lost roughly 854 frames/s over the final two seconds of the supplied
log, and bus 1 roughly 123 frames/s over the same interval.

At 76.790 s the task watchdog reported that **IDLE1 had not run in time**.
The running tasks were `wifi` on CPU 0 and `mcp251xfd` on CPU 1. The CPU 1
backtrace sampled the worker in polling SPI while draining the transmit event
FIFO. This establishes idle starvation. A single sampled stack does not
establish that one SPI transfer hung.

Pinning the forwarding task and MCP worker to the same CPU removed the
parallel execution available in the first pass. That is a plausible cause of
the new normal-use regression, but the scheduling and submission-ring changes
were not tested separately. There is also no fairness guarantee in the old
worker loop: `ulTaskNotifyTake()` returns immediately when notified, and the
worker notifies itself again while INT is low. Sustained external submissions
can likewise keep notifications pending. A bounded service pass alone does
not bound the length of a continuously runnable sequence of passes.

## Follow-up: restore task placement and let the worker block

The receive/forward task is restored to the first-pass configuration:
unpinned, priority 7. The MCP worker remains at priority 20 on core 1. The
separate TX submission ring remains; its independent hardware performance
effect has not been established.

The worker now checks elapsed busy time between complete service passes. After
8 ms without reaching its wait for new work, it releases both mutexes and
blocks for one tick (1 ms at the current tick rate). Pending notifications
remain available, and the hardware FIFOs continue operating during the pause.
An ordinary `taskYIELD()` would not admit lower-priority tasks while the worker
remains ready. The watchdog stays enabled. This is an initial scheduling
budget to test, not a measured optimum or a timeout on individual SPI calls.
Time spent in preemption or a long service pass can exceed the nominal budget.

A rate-limited WARN line reports `busy slice` elapsed time, pass count,
longest service pass, last `CiINT` snapshot and cumulative hardware RX overflow
observations. These help distinguish frequent short passes from long service
passes and detect loss moving into the hardware FIFO. Hardware overflow flags
do not count the exact number of lost frames.

A new host test forces INT continuously low, checks repeated nonzero blocking
delays with neither driver mutex held, then verifies TX completion, RX delivery,
queue status and deletion while INT remains asserted. It fails with the
previous worker loop and passes with the follow-up. The full core/adapter and
ASan/UBSan tests pass, and `v300`, `proto` and `eb-fd` all build successfully.
The new retest image is copied to `build.eb-fd/wican-eb-fd-worker-fairness.bin`;
the earlier `wican-eb-fd-wifi-load.bin` copy is the failed second-pass image.
These tests do not model ESP32 scheduling or establish hardware throughput.
Physical confirmation is still pending.

Retest normal MITM forwarding, AP connection, repeated page loads and sustained
SavvyCAN streaming under the same CAN workload. Record per-bus RX queue-drop
counter deltas, forwarding/TX drops, host-stream drops and any controller
fault. Include the new `busy slice` warnings and any watchdog report. A
reduction in one drop counter must not merely move the loss to another queue.

## Hardware retest of worker fairness

The supplied log from `wican-eb-fd-worker-fairness.bin` contains no software
queue-drop warnings before Wi-Fi connection and no watchdog report. The
pre-connection excerpt spans approximately 1.8–29.2 s. This is encouraging,
but does not establish a long-run lossless rate or independently isolate the
effects of restoring task placement and adding the worker pause.

During connection, shared software RX queue counters reached 466 on bus 0
and 30 on bus 1. Bus-1 TX-pool drops were also reported, reaching 137 in the
last supplied TX warning. `HW RX overruns` remained zero in every supplied
worker warning. The later excerpt ends around 64.6 s with no additional
queue-drop warnings after the TX warning around 54.4 s. The remaining observed
loss is therefore bursty around connection, rather than the previous steady
pre-connection overflow. The excerpt does not demonstrate lossless page loads
or sustained SavvyCAN streaming.

`INT=0010` is the transmit event FIFO interrupt, and `0012` adds the receive
FIFO interrupt. They are expected during forwarding. The logged busy slices
were approximately 8.0–8.6 ms, with 11–24 passes and longest passes up to
approximately 1.5 ms. The current diagnostic rate limit sampled one slice
per second; those values cannot exclude a longer scheduling stall elsewhere
in that second. No controller RX overflow was observed, but software queues
and forwarding still lost frames.

Source review also found that the bridge's existing 2-tick TX-slot timeout
activates a 100 ms fast-drop interval. A temporary scheduling stall can thus
cause forwarding drops after RX servicing resumes. The log does not establish
how much of the 137 TX drops came from this policy. The policy and queue sizes
are left unchanged in the next image while controller overhead is reduced.

## Next pass: reduce TX SPI overhead and retain worst timing samples

Two redundant reads are removed from the common transmit path:

- Read TX status and UA together when a refill is pending, then reuse that
  snapshot for the first refill. Subsequent FIFO advances and one-shot resets
  obtain fresh snapshots. Error/status checks still run on every pass.
- Stop draining TEF after all loaded frames have completed, avoiding another
  read merely to confirm emptiness. An unexpected extra event remains pending
  and is rejected on the next pass by the existing ownership/sequence checks.

Measured with the same host SPI simulator workloads:

| Service workload | Previous image | Next image |
| --- | ---: | ---: |
| Idle | 3 | 3 |
| One TX submission | 6 | 5 |
| One TX submission plus completion | 10 | 8 |
| One TX submission, RX frame and completion | 14 | 12 |

These are SPI transaction counts, not CPU speedups or hardware loss rates.
The TX queue limit, hardware FIFO depths, SPI rate, CPU frequency, task
priorities and worker pause budget remain unchanged.

The WARN-level `service window` diagnostic replaces `busy slice`. It retains
the worst gap between service starts, longest pass and longest busy interval
over each reporting window, including intervals when rate limiting suppresses
output. It also reports the window duration, successful TX completions, RX
callback deliveries, pass count and pause count. RX callback delivery can
still be followed by a software queue drop. Gap includes the ordinary idle
poll wait, scheduling/mutex waits and log overhead, so compare it with the
pre-connection baseline; it is not a direct measurement of SPI time.

The existing tests cover FIFO wrap, repeated one-shot failures/refills and
failure at each SPI operation. A new test injects an extra TEF entry after the
last valid completion and checks that it faults on the next pass without
returning ownership twice. The new SPI budgets also cover completion/refill
without RX, matching the TEF-heavy workload indicated by the logs.

Core and adapter tests, ASan/UBSan, and all three firmware builds (`v300`,
`proto`, `eb-fd`) pass. The next hardware test image is
`build.eb-fd/wican-eb-fd-tx-overhead.bin`; the previous fairness image is
preserved separately. Repeat forwarding before connection, AP joining, page
loads and SavvyCAN streaming, capturing the `service window` lines and every
RX/TX/host-stream drop counter. Hardware improvement from this pass has not
yet been confirmed.

## TX-overhead retest: worker progresses through the RX drop burst

The next supplied post-connection log, approximately 10.8–17.8 s after boot,
shows 1,968–2,060 successful MCP transmissions and 171–177 RX callbacks per
reporting window (1,000–1,002 ms). The worker's worst start-to-start gaps were
5.3–7.5 ms, longest passes 1.0–1.6 ms, and longest busy intervals 8.2–8.5 ms.
Hardware RX overflow observations remained zero. Each window contained
9–15 planned one-tick pauses.

Shared software RX drop counters nevertheless reached 352 for bus 0 and 51
for bus 1 around 11–12 s, with at least one bus-1 TX-pool timeout. No further
drop warning appears in the remainder of the excerpt. These results do not
establish a long SPI-worker stall as the cause: the worker continues servicing
the controller while the shared software queue overflows. The RX consumer can
be delayed by scheduling, its state-machine mutex, forwarding backpressure or
other per-frame work; its latency was not yet directly measured.

## Next pass: prioritize the shared consumer on EB-FD

The shared RX/forward task previously ran unpinned at priority 7. The local
ESP-IDF revision defines WPA3 AP authentication at priority 19 in
`components/wpa_supplicant/esp_supplicant/src/esp_wpa3_i.h`; the MCP worker runs
at 20. WiCAN enables mixed WPA2/WPA3 AP mode. Authentication can therefore
outrank the consumer while the MCP worker continues running. The log does
not identify the client's negotiated authentication mode, or prove which
task/lock delayed the consumer.

This pass raises the EB-FD consumer to priority 21 while keeping it unpinned.
It can run ahead of authentication and the MCP worker, and either core remains
available. The earlier failed experiment pinned it to core 1 below the MCP
worker; this configuration preserves cross-core execution and the worker's
existing blocking pause. The asynchronous TX submission path is required so a
higher-priority consumer does not synchronously wait for every SPI service pass.

The consumer also checks elapsed time between complete frames while draining
a backlog, blocking for one tick after an 8 ms burst. No state-machine or TX
lock is held during that pause. This prevents a permanently nonempty queue
from keeping this task continuously runnable. The time budget includes waits
and preemption and cannot bound an individual frame's processing time. The
watchdog remains enabled. Proto/v300 keep their original receive-task priority
and loop behavior. SPI/CPU clocks, controller servicing, queue capacities and
the bridge's TX backpressure policy are unchanged from the previous image.

For direct consumer evidence, EB-FD RX queue entries now carry an enqueue
timestamp. `RX scheduling` warnings retain the maximum queue residence and
dequeue gap across each reporting interval, logging at most once per second
when residence exceeds 20 ms. The gap includes normal idle time and all work
between dequeues; a large residence time confirms a waiting software backlog
but does not by itself distinguish CPU scheduling from a blocking call. The
depth in the log is a current snapshot. This adds 2 KiB of timestamp storage
to the existing 256-entry queue without increasing its frame capacity.

All three firmware variants build, and the existing application host tests
(state machine, precondition, ISO-TP and popup handling) pass. The MCP driver
is unchanged from the previously tested image. These checks do not exercise
ESP32 priority/preemption behavior or establish that the Wi-Fi drops are fixed.
The new image is `build.eb-fd/wican-eb-fd-forward-priority.bin`; the previous
`wican-eb-fd-tx-overhead.bin` copy is preserved for comparison. Retest normal
forwarding, AP association, page loads and SavvyCAN streaming with the same
traffic, capturing `RX scheduling`, `service window`, RX/TX/host-stream drops
and any watchdog report. Physical confirmation is pending.

## Accepted hardware result and diagnostic cleanup

The user reports the forwarding-priority image has **no drops while joining
Wi-Fi or using the HTTP interface**, with a small drop burst when connecting
SavvyCAN. The supplied log spans approximately 1.8–34.8 s. After the initial
window it shows about 2,040–2,070 successful MCP transmissions per second and
roughly 170 RX callbacks per second. No hardware RX overflow, shared RX queue
drop, `RX scheduling` warning or watchdog report appears in this excerpt.
The remaining warning is bus-1 TX pool exhaustion, totaling **76 dropped
frames** at approximately 32 s. This does not establish lossless forwarding
during every SavvyCAN workload or a long-run throughput limit.

The user accepts this result for now and requests no further tuning. Retain
the asynchronous TX submission queue, reduced SPI reads, unpinned priority-21
EB-FD consumer, and the 8 ms/one-tick busy budgets for consumer and MCP worker.
The bridge's existing two-tick TX wait and 100 ms fast-drop fallback remain
unchanged; their contribution to the remaining SavvyCAN burst is unmeasured.

`CONFIG_MCP251XFD_PERF_DIAGNOSTICS` now defaults off. It compiles out the
`service window` and `RX scheduling` logs together with their counters,
per-pass diagnostic timing and RX queue timestamps (saving 2 KiB of queue
storage). The timers needed for the busy budgets still run. Ordinary queue
drop and controller fault warnings remain active. The option can be enabled
under `MCP2518FD driver` in `menuconfig` for later work.

Final validation: all three firmware variants build with diagnostics disabled;
their binaries omit the throughput diagnostic strings and retain RX/TX drop
warnings. Core/adapter host tests pass with diagnostics both disabled and
enabled, and ASan/UBSan pass with them disabled. The EB-FD application CAN
source also compiles with instrumentation enabled. The application host suite
passed after the forwarding-priority change. The final diagnostic-off image
has not received a separate physical retest.
