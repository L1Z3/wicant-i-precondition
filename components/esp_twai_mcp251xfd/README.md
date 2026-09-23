# esp_twai_mcp251xfd

A TWAI node (the `esp_driver_twai` interface) for an MCP2517FD/MCP2518FD on
SPI, built on Zephyr's MCP251xFD driver. Classical CAN only. WiCAN uses it for
bus 1 on `eb-fd`.

## Layout

- `zephyr/`: Zephyr v4.4.2 sources, unmodified apart from the local patches
  below. Provenance is in `zephyr/README.md`.
- `compat/`: the Zephyr APIs those sources use, implemented on ESP-IDF and
  FreeRTOS (kernel objects, SPI, GPIO, logging, utility macros, syscall
  wrappers). `compat/autoconf.h` holds the driver's Zephyr config values.
- `esp_twai_mcp251xfd.c`: the TWAI adapter. It compiles the driver source in
  directly and fills in the device and config that Zephyr would build from
  devicetree.

## Local patches

In `zephyr/drivers/can/can_mcp251xfd.{c,h}`, each marked `LOCAL PATCH`:

1. **TX FIFO instead of TXQ.** The TXQ sends the lowest pending CAN ID first,
   which would reorder forwarded frames. Linux's driver also uses FIFO 1.
2. **One RX object per read.** Erratum DS80000789 #6: `FIFOSTA.FIFOCI` reads
   can be corrupted with a valid CRC, so batching on it can deliver stale
   objects. RX now uses the same status/`UA` path as TEF.
3. **1 ms interrupt back-off instead of 10 ms.** The driver pauses after ten
   passes with INT still asserted, which sustained forwarding triggers; 10 ms
   without TEF servicing exhausts the TX mailboxes.
4. **`get_state()` returns read errors.** Upstream returns 0, and the
   state-change handler then reports an uninitialized state.

To update Zephyr, copy the files listed in `zephyr/README.md` from the new
release, reapply the patches, and check the driver for new API use.

## Behaviour

- **One node.** The first `twai_new_node_mcp251xfd()` initializes the
  controller and starts the driver's thread. Zephyr drivers have no teardown,
  so `twai_node_delete()` keeps the controller, and the next call applies new
  bit timing, mode and filters. If initialization fails, the node stays
  unavailable until restart.
- **Sleep.** `twai_node_delete()` puts the controller in Sleep mode: the
  oscillator stops (15 uA typical instead of ~15 mA) and registers and RAM are
  kept, so the next creation wakes it, waits up to 3 ms for the oscillator and
  carries on. Low Power Mode would draw 4 uA but loses the configuration,
  which the driver cannot rebuild. Bus activity does not wake the controller.
  Interrupt enables are cleared while asleep so INT is released. The
  controller has its own power, so it can still be asleep after an
  `esp_restart()`; the first creation wakes it before initializing.
- **Threads.** A level-triggered INT interrupt wakes the driver's thread
  (priority 20, core 1), which services RX, TX completion and errors and runs
  all callbacks. `transmit()` writes the frame over SPI from the caller's task
  (three transfers) under the driver's mutex.
- **TX.** Up to 32 frames are in flight in the controller's TX FIFO.
  `on_tx_done` returns the submitted frame pointer once its TEF entry arrives.
  `twai_node_disable()` aborts queued frames and reports them as failed before
  returning; so does bus-off.
- **RX filters.** By default all standard and extended frames are accepted.
  Mask filter 0 replaces that. Upstream's filter removal also clears the next
  three filters' enable bits, so the adapter always removes every filter before
  adding new ones.
- **Not supported:** one-shot transmission, CAN FD, timestamps, range or dual
  filters, `get_info`, `recover`, `reconfig_timing` and
  `transmit_wait_all_done`.
- **SPI.** Every transfer is at most 18 bytes, so the bus runs without DMA. The
  node reserves the bus. Initialization uses ESP-IDF's SPI driver; after that,
  transfers program the peripheral directly through ESP-IDF's low-level HAL
  (`hal/spi_ll.h`), reusing the bus configuration the driver set up. This
  roughly halves the cost of each transfer, but depends on ESP-IDF internals
  and on the node being the only device on its SPI bus.
