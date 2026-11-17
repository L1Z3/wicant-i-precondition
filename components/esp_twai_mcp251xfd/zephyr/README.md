# Vendored Zephyr sources

Copied from [Zephyr](https://github.com/zephyrproject-rtos/zephyr) v4.4.2
(commit `dccb09599635bdff17633fa7e9dab014b91dce90`).

Local modifications are marked `LOCAL PATCH`; ../README.md describes them.

## Upgrading

Commit `820c2d9` in this repository imported these files unmodified, so
the local patches are

    git diff 820c2d9 -- components/esp_twai_mcp251xfd/zephyr

If history has been rewritten, diff against the upstream files at Zephyr
commit `dccb0959` instead (`drivers/can/can_mcp251xfd.{c,h}`,
`drivers/can/can_common.c`, `subsys/crc/crc16_sw.c`,
`include/zephyr/drivers/can.h`). Every hunk in
`can_mcp251xfd.{c,h}` carries a `LOCAL PATCH` comment; the other files are
unmodified.

To upgrade, copy the new upstream files over these, commit them unmodified,
then reapply that diff and update the Zephyr version and commit above and
the import commit here. Also check the shim in `../compat`, which follows
these files' APIs:

- `compat/zephyr/syscalls/can.h` forwards every `__syscall` in `can.h`; a
  missing one shows up as a "declared 'static' but never defined" warning.
- `compat/autoconf.h` holds the driver's Kconfig values.
- The rest of `compat/zephyr` implements the kernel, SPI, GPIO and other
  APIs the driver calls.
