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

The adapter test compiles the actual ESP-IDF adapter against pthread-backed
platform stubs. It checks worker/ISR separation, queue backpressure, original
frame pointers, concurrent disable while a callback is active, a waiting sender
during disable, controller disconnection, and cleanup after partially completed
creation. Resource counters check that tasks, semaphores, event groups, and SPI
devices are released. These are host tests, not measurements on real hardware.

## Bench checklist — pending physical prototype

Use an isolated, correctly terminated Classical CAN test bus and a second
CAN adapter that can ACK frames. Record firmware commit, oscillator, SPI rate,
pin mapping, nominal bitrate, test duration, observed loss, and errors.

1. Confirm the provisional board settings in `main/hw_config.h`: SPI2 SCLK 17,
   MOSI 16, MISO 15, CS 18, INT 7; on-chip TX 2/RX 1; ESP32-S3 N16R8; LEDs and
   VBAT divider. Confirm the 40 MHz oscillator and transceiver standby wiring.
   MCP2518FD has no GPIO reset; GPIO 8 is not driven for reset.
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
   creation and controller-fault cleanup.
9. Sustain simultaneous RX/TX on both CAN buses with Wi-Fi/GVRET streaming.
   Measure ordering, loss/overflow, latency, and heap over an extended run.
   Compare with the existing proto board under the same workload.

Full CAN FD traffic is outside this implementation. In Normal CAN 2.0 mode the
controller can emit error frames on FD traffic; this checklist assumes a
Classical-only bus. The register behavior and SPI limit are described in the
[MCP2518FD datasheet](https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/External-CAN-FD-Controller-with-SPI-Interface-DS20006027B.pdf)
and the [MCP25XXFD reference manual](https://ww1.microchip.com/downloads/en/DeviceDoc/MCP25XXFD-FRM,-CAN-FD-Controller-Module-DS20005678D.pdf).
