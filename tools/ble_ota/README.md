# Firmware update over BLE

`ATXOB` installs firmware over the WiCAN's BLE link, for when its Wi-Fi isn't reachable: with the car on, a head unit running wireless CarPlay or Android Auto keeps the phone on its own Wi-Fi, so the web page's update can't hold a connection.

## Protocol

On the command characteristic (service `FEE0`, characteristic `FEE1`), the one the ELM327 commands use:

| Send | Reply |
|---|---|
| `ATXOB` | `XOB:1`: this firmware supports it |
| `ATXOB<size>` | `XOB:READY`, or `XOB:ERR,SIZE` / `XOB:ERR,BEGIN` |
| `<size>` bytes of the `.bin`, as writes with response | none; `$XOB:ERR,<reason>` if a write fails |
| `ATXOBEND` | `XOB:DONE` (reboots 500 ms later), or `XOB:ERR,IMAGE` / `XOB:ERR,VARIANT` / `XOB:ERR,INCOMPLETE` |

- Writes can be up to 244 bytes (one ATT payload at a 247-byte MTU). Smaller writes work, just more slowly.
- While image bytes are expected, every write on the characteristic goes to flash, so send nothing else until the last byte is written.
- Dropping the link abandons the transfer. The running firmware is untouched until `ATXOBEND` passes.
- The image goes through `esp_ota_*` like the web upload: it is checked before it can boot, and an image for another board variant is refused.

## Desktop client

`ble_ota.py` does the above from a computer:

    pip install bleak
    python3 tools/ble_ota/ble_ota.py wican-fw_v300_<version>.bin

Pair the computer with the WiCAN first (it asks for the BLE passkey from the web page).
