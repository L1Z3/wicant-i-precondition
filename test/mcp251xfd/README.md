# MCP2518FD validation

Run from the repository root:

```sh
make -C test/mcp251xfd
make -C test/mcp251xfd sanitize
make -C test/host
./build.sh all
```

The core test uses the real vendored driver through an independent SPI register
and RAM model. It covers nominal timing, initialization, FIFO allocation and
submission order, sequence wrap, original completion tokens, one-shot failures,
standard/extended IDs, RTR, filtering register contents, RX overflow, FD rejection,
bus-off (including automatic hardware recovery before polling), TEF corruption,
SPI failure, mode timeout, repeated enable/disable, and failed initialization.
It also models LPM wake-up clearing registers/RAM, an unusable first SPI reply,
and delayed oscillator readiness, then verifies configuration and TX after
recreating the software context without resetting the simulated hardware.

The adapter test compiles the actual ESP-IDF adapter against pthread-backed
platform stubs. It checks worker/ISR separation, queue backpressure, original
frame pointers, concurrent disable while a callback is active, a waiting sender
during disable, controller disconnection, and cleanup after partially completed
creation. Resource counters check that tasks, semaphores, event groups, and SPI
devices are released. Deletion/recreation also checks LPM entry, no subsequent
worker SPI access, and CS hold/release across device removal and creation.
These are host tests, not measurements on real hardware.

## Bring-up diagnostics

Capture the `mcp251xfd: enabled` and `readback` lines and the first complete
fault report. At the configured 40 MHz clock and 500 kbit/s, `NBTCFG` should
read `00440909`; `OSC` bits 0 and 4 should be clear (PLL/divider disabled).
`CON` bits 23:21 should be 6 in normal Classical CAN mode. The register-valid
mask should be `f`; bits 0..3 correspond to CON/NBTCFG/OSC/IOCON respectively.

In the service path, error 10 (`ERR__NOT_READY`) means the driver observed
`CiTREC.TXBO` or `CiBDIAG1.TXBOERR`. It does not identify the physical cause.
The log now includes TEC/REC and ACK, BIT0, BIT1, FORM, STUFF, and CRC flags,
including flags from earlier polls. The captured mode matters: configuration
mode itself sets TXBO, so registers must be captured before fault cleanup.

The supplied EB-FD firmware identifies ESP32 GPIO 11 -> SN65HVD233/U2 RS and
GPIO 12 -> TCAN3413/U4 STB, each with a 10 kOhm pull-up. Both must be low during
operation and high when their respective bus is disabled. GPIO 8 is unused;
MCP2518FD GPIO0/GPIO1 remain inputs. Verify these levels on the physical board,
along with CAN connector routing. A quiet log in parallel mode does not
demonstrate a working bus-1 path; verify actual RX in SavvyCAN as well as TX.

## Bench checklist — pending physical prototype

Use an isolated, correctly terminated Classical CAN test bus and a second
CAN adapter that can ACK frames. Record firmware commit, oscillator, SPI rate,
pin mapping, nominal bitrate, test duration, observed loss, and errors.

1. Verify the supplied board settings in `main/hw_config.h`: SPI2 SCLK 17,
   MOSI 16, MISO 15, CS 18, INT 7; on-chip TX 2/RX 1; 40 MHz crystal;
   standby controls 11/12; LEDs and VBAT divider. Confirm the provisional
   ESP32-S3 N16R8 module capacity separately. GPIO 8 is not driven for reset.
2. Build/flash `eb-fd`. Check reset/configuration at 1 MHz and operation at
   10 MHz with a logic analyzer. Confirm bus 0 stays usable if bus 1 hardware
   is missing or initialization fails.
3. Exercise bus 1 in internal loopback using the CAN API with normal mode off.
   Verify standard and extended ID boundaries, DLC 0–8, and RTR frames. Check
   that returned frames match and TX capacity returns to 32 after completion.
4. With a peer attached, test supported nominal bitrates, especially 500 kbit/s,
   800 kbit/s, and 1 Mbit/s. Capture a descending-ID burst to verify FIFO order.
   Exercise exact-ID and masked filters for standard and extended frames,
   confirming there is no accept-all bypass.
5. Enable listen-only and confirm reception without ACK/transmission. Return to
   normal mode and verify continued bidirectional traffic.
6. Disconnect the ACKing peer. With auto-retry disabled, send more than 32
   frames and confirm failed completions keep freeing slots. Reconnect the peer
   and verify transmission resumes. With auto-retry enabled, check backpressure
   rather than false successful completions.
7. Induce bus-off with suitable CAN fault-injection equipment; confirm one
   recovery cycle retires pending frames, recreates bus 1, and allows fresh TX.
   Exercise errors that occur around enable and during simultaneous RX/TX.
8. Repeatedly disable/enable and change bitrate/mode while traffic is flowing.
   Check for stale callbacks, duplicate completions, leaked slots, watchdog
   resets, or heap loss. Disconnect/reconnect SPI hardware to exercise failed
   creation and controller-fault cleanup. Check that standby returns high on a
   failed enable and goes low again after successful recovery.
9. Sustain simultaneous RX/TX on both CAN buses with Wi-Fi/GVRET streaming.
   Measure ordering, loss/overflow, latency, and heap over an extended run.
   Compare with the existing proto board under the same workload.
10. Exercise the existing voltage-triggered sleep and wake path. Check GPIOs
    11/12 high and CS 18 continuously high during sleep, with no SPI polling
    after the LPM request. Measure supply current; do not probe sleeping
    controller registers over SPI, since that wakes it. On wake, verify both
    transceivers return low, the controller is reconfigured, and RX/TX resume.

Full CAN FD traffic is outside this implementation. In Normal CAN 2.0 mode the
controller can emit error frames on FD traffic; this checklist assumes a
Classical-only bus. The register behavior and SPI limit are described in the
[MCP2518FD datasheet](https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/External-CAN-FD-Controller-with-SPI-Interface-DS20006027B.pdf)
and the [MCP25XXFD reference manual](https://ww1.microchip.com/downloads/en/DeviceDoc/MCP25XXFD-FRM,-CAN-FD-Controller-Module-DS20005678D.pdf).
