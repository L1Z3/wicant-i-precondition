# esp_twai_mcp251xfd

A TWAI node (the `esp_driver_twai` interface) for an MCP2517FD/MCP2518FD on
SPI, built on Zephyr's MCP251xFD driver. Classical CAN only. WiCAN uses it for
bus 1 on `eb-fd`.

## Layout

- `zephyr/`: Zephyr v4.4.2 sources, unmodified apart from the local patches
  below.
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
2. **One RX object per read.** [Erratum DS80000789 #6][errata]: reading an RX
   FIFO's `FIFOSTA.FIFOCI` while a frame arrives can return a wrong value with
   a matching CRC, so batching on it can deliver already-consumed objects.
   Rather than Microchip's workaround of reading it repeatedly and comparing,
   RX no longer uses `FIFOCI` and takes the same one-object status/`UA` path
   as TEF.
3. **Interrupt back-off only without progress.** The driver pauses for 10 ms
   after ten passes with INT still asserted, meant for a flag that can't be
   cleared. With one RX object per pass, sustained traffic triggered it too,
   and 10 ms without TEF servicing exhausts the TX mailboxes. Passes that read
   an RX or TEF object now reset the count. A failed `CiINT` read counts as a
   pass without progress instead of retrying at once, which spun the
   driver's thread while reads kept failing.
4. **`get_state()` returns read errors.** Upstream returns 0, and the
   state-change handler then reports an uninitialized state. A failed read
   also keeps the interrupt loop running, under the back-off, until a read
   succeeds or the controller is stopped: `CERRIF` is already cleared by
   then, so the state change would otherwise be lost.
5. **Bounded abort wait in `stop()`.** Upstream polls for `ABAT` to clear
   without a limit, hanging the caller if the controller never clears it.
   The wait now times out after 200 ms and fails the stop.
6. **INT's interrupt enabled after controller setup.** Upstream enables it
   before resetting the controller, which may still be asserting INT after an
   MCU restart; the interrupt thread then accesses the controller alongside
   initialization, which doesn't take the driver's mutex.

[errata]: https://ww1.microchip.com/downloads/aemDocuments/documents/APID/ProductDocuments/Errata/MCP2518FD-Silicon-Errata-and-Data-Sheet-Clarification-DS80000789.pdf

## Behaviour

- **One node.** The first `twai_new_node_mcp251xfd()` initializes the driver
  and starts its thread. Zephyr drivers have no teardown, so
  `twai_node_delete()` keeps the node, and every call, the first included,
  resets the controller and redoes the driver's register setup
  (`controller_reset()`, retried like Linux's soft reset) before applying bit
  timing, mode and filters. Only a failure to set up the INT pin leaves the
  node unavailable until restart. If `twai_node_disable()` fails to stop the
  controller, `twai_node_delete()` resets it and still releases the node.
- **Sleep.** `twai_node_delete()` puts the controller in Low Power Mode:
  4 uA typical instead of ~15 mA, losing registers and RAM, which the next
  reset rebuilds. Any SPI access wakes it, so it waits for the driver's
  thread to finish first; bus activity does not wake it. Unlike Sleep mode,
  which keeps the registers, waking doesn't clear `OSCDIS`, so [erratum
  DS80000789 #7][errata] (a wake that doesn't last) doesn't apply.
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
