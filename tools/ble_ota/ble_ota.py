#!/usr/bin/env python3
"""Install WiCAN firmware over Bluetooth LE (ATXOB; see main/ble_ota.c).

For when the WiCAN's Wi-Fi isn't reachable, for example with the car on and
the head unit holding the phone's Wi-Fi for wireless CarPlay or Android
Auto. Needs Python 3.9+ and bleak:

    pip install bleak
    python3 tools/ble_ota/ble_ota.py wican-fw_v300_<version>.bin
    python3 tools/ble_ota/ble_ota.py --address AA:BB:CC:DD:EE:FF firmware.bin

The WiCAN must have BLE enabled and be paired with this computer (it asks
for the BLE passkey set on its web page). Install the .bin, not -full.bin.
The image must be built for the same board variant as the running firmware.
"""

import argparse
import asyncio
import sys

from bleak import BleakClient, BleakScanner

COMMAND_CHAR = "0000fee1-0000-1000-8000-00805f9b34fb"


class Link:
    """Commands and replies on the WiCAN's command characteristic."""

    def __init__(self, client):
        self.client = client
        self.buffer = ""
        self.replies = asyncio.Queue()
        self.failure = None

    def on_notify(self, _sender, data):
        self.buffer += data.decode("ascii", errors="replace")
        while "\r" in self.buffer:
            line, self.buffer = self.buffer.split("\r", 1)
            line = line.strip(" >\n")
            if not line:
                continue
            if line.startswith("$XOB:ERR"):
                self.failure = line[1:]
            elif not line.startswith("$"):
                self.replies.put_nowait(line)

    async def command(self, text, timeout=5.0):
        data = (text + "\r").encode("ascii")
        # Commands go in pieces of 60 bytes or fewer: the WiCAN joins them at the CR.
        for i in range(0, len(data), 60):
            await self.client.write_gatt_char(COMMAND_CHAR, data[i:i + 60], response=True)
        deadline = asyncio.get_running_loop().time() + timeout
        while True:
            remaining = deadline - asyncio.get_running_loop().time()
            if remaining <= 0:
                raise TimeoutError(f"no reply to {text}")
            reply = await asyncio.wait_for(self.replies.get(), remaining)
            if reply.upper().startswith("XOB"):
                return reply


async def find_wican(address):
    if address:
        return address
    print("Looking for a WiCAN…")
    devices = await BleakScanner.discover(timeout=8.0, return_adv=True)
    for device, adv in devices.values():
        services = [s.lower() for s in adv.service_uuids]
        if "0000fee0-0000-1000-8000-00805f9b34fb" in services or (device.name or "").upper().startswith("WIC"):
            print(f"Found {device.name or 'WiCAN'} at {device.address}")
            return device.address
    sys.exit("No WiCAN found. Check BLE is enabled on it, or pass --address.")


async def install(path, address, chunk):
    image = open(path, "rb").read()
    address = await find_wican(address)
    async with BleakClient(address) as client:
        link = Link(client)
        await client.start_notify(COMMAND_CHAR, link.on_notify)

        probe = await link.command("ATXOB")
        if not probe.upper().startswith("XOB:1"):
            sys.exit(f"This firmware can't install over BLE ({probe}). Update it over Wi-Fi once.")

        ready = await link.command(f"ATXOB{len(image)}")
        if "READY" not in ready.upper():
            sys.exit(f"The WiCAN couldn't start the update: {ready}")

        mtu_payload = max(20, client.mtu_size - 3)
        size = min(chunk, mtu_payload, 240)
        print(f"Sending {len(image)} bytes in {size}-byte writes…")
        sent = 0
        last_shown = -1
        while sent < len(image):
            if link.failure:
                sys.exit(f"The WiCAN stopped the transfer: {link.failure}. It keeps its current firmware.")
            piece = image[sent:sent + size]
            await client.write_gatt_char(COMMAND_CHAR, piece, response=True)
            sent += len(piece)
            percent = sent * 100 // len(image)
            if percent != last_shown:
                print(f"\r{percent}%", end="", flush=True)
                last_shown = percent
        print()

        print("Checking the image…")
        done = await link.command("ATXOBEND", timeout=15.0)
        if "DONE" in done.upper():
            print("Installed. The WiCAN restarts on the new firmware.")
        elif "VARIANT" in done.upper():
            sys.exit("Refused: the image is for another board variant. The WiCAN keeps its current firmware.")
        elif "IMAGE" in done.upper():
            sys.exit("Refused: the image failed its check. The WiCAN keeps its current firmware.")
        else:
            sys.exit(f"The WiCAN couldn't finish the update: {done}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("firmware", help="the .bin to install (not -full.bin)")
    parser.add_argument("--address", help="the WiCAN's BLE address (macOS: its identifier); scans when left out")
    parser.add_argument("--chunk", type=int, default=240, help="bytes per write, at most 240 (default 240)")
    args = parser.parse_args()
    asyncio.run(install(args.firmware, args.address, args.chunk))


if __name__ == "__main__":
    main()
