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

## Remaining regression: joining the access point

With the same optimized firmware, attempting to join WiCAN's AP starts RX
queue drops and the connection times out. The user reports that prototype 1
handles this without problems, and EB-FD could accept AP connections before
this optimization pass despite its continuous CAN drops.

The supplied excerpt shows intermittent increases rather than the earlier
continuous loss rate. Bus 0 rises from 305 drops at 75.606 s to 498 at
90.004 s; bus 1 rises from 33 at 80.053 s to 48 at 89.055 s. Repeated
`wifi:rm mis` messages accompany the attempts. These messages also occurred
in the earlier pre-optimization log, so their presence alone does not
establish the cause of the new connection failure. No controller fault or
bus-off is shown in this excerpt.

The AP regression remains unresolved in this optimization snapshot. The
logs establish a correlation with connection attempts, not whether the
stall is in Wi-Fi authentication, DHCP, task scheduling, or SPI servicing.
Any follow-up must preserve the normal-use improvement and test AP joins
under the same CAN load. Record whether association and IP assignment
succeed, RX queue counter deltas, and any controller faults. Host simulation
does not reproduce ESP32 scheduling or Wi-Fi authentication timing.
