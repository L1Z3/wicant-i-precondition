# What is this?

This is a fork of the [WiCAN firmware repository](https://github.com/meatpihq/wican-fw) designed to add custom CAN-based buttons. For now, the button added is a manual preconditioning button for first-generation E-GMP cars (2021-2024 Hyundai Ioniq 5 and EV6, 2023-2025 Ioniq 6). This is only possible through direct access to CAN busses with a [custom harness](https://github.com/tylerharvey/Ioniq5_CAN/wiring_harness/) that you can build or [buy](https://electroniqbuttons.com). See [our project coordination repository](https://github.com/tylerharvey/Ioniq5_CAN) for more background.

# [WiCAN Documentation](https://meatpihq.github.io/wican-fw/) | [Firmware updates](https://github.com/L1Z3/wicant-i-precondition/releases/) | [Fluxer server](https://fluxer.gg/w0OpDJjG)

# Building

1. Clone this repo recursively: `git clone --recursive https://github.com/L1Z3/wicant-i-precondition.git`
2. Install ESP-IDF: `cd wicant-i-precondition/esp-idf && ./install.sh`
3. Export the environment: `. ./export.sh`
4. Open project and build: `cd .. && ./build.sh v300`
5. Flash: use http interface or if using USB: `idf.py -B build.v300 flash`

Select a hardware variant with `./build.sh v300`, `./build.sh proto`, or
`./build.sh eb-fd`; `./build.sh all` builds all three into separate
`build.<variant>` directories. Use that same directory with `idf.py -B` when
flashing. Direct `idf.py` builds default to `proto`; override with
`-DHARDWARE_VER_NAME=<variant>`.

| Variant | Target | External CAN controller | Hardware version |
| --- | --- | --- | --- |
| `v300` | ESP32-C3 | None | `WiCAN-OBD` |
| `proto` (formerly `custom`) | ESP32-S3 | MCP2515, 8 MHz oscillator | `WiCAN-PROTO` |
| `eb-fd` | ESP32-S3 | MCP2518FD, 40 MHz oscillator | `WiCAN-EB-FD` |

`eb-fd` uses the MCP2518FD for Classical CAN on bus 1 through the
[esp_twai_mcp251xfd component](components/esp_twai_mcp251xfd/README.md).
The 40 MHz controller oscillator is configured without PLL or division; SPI
runs at 10 MHz after initialization. The ESP32-S3 module configuration, GPIO
mapping, SPI speed, LEDs, and battery sensing provisionally match `proto`
pending schematic confirmation. The MCP2515 reset GPIO is not used on `eb-fd`.
CAN FD payloads are not supported. This integration has build and host-test
coverage; validation on the physical prototype is still required.

# WiCAN-OBD-C3 Information

The WiCAN original shipped with the beta preconditioning kit is a powerful ESP32-C3-based CAN adapter for car hacking and general CAN-bus development. WiCAN connects to your existing Wi-Fi network and any device on that network, where it allows you to configure Wi-Fi and CAN settings through a built-in web interface. The WiCAN has a power-saving mode that detects when the voltage drops under 13 V or some other preset value. When this power-saving mode is engaged, WiCAN is capable of entering sleep mode, which drops current consumption below 1 mA.

WiCAN is a simple, ready-to-use solution for CAN-bus development and hacking. It accelerates development by providing vehicle-diagnostic APIs and libraries in various languages and for various operating systems. WiCAN works with a large array of pre-existing vehicle-diagnostic libraries, including RealDash, SavvyCAN, BUSmaster, python-can/SocketCA, and more. APIs are also available for LabView, C#, VB.Net, Delphi, and Python in case you’re writing your own software.

## Description
![image](https://user-images.githubusercontent.com/94690098/231444160-08842087-55ad-4165-8291-b379da63aeeb.png)

WiCAN-OBD will be of great interest to car enthusiasts and tinkers who want to modernize or customize the head-unit displays in their cars using RealDash. Check out some examples of the available graphic interfaces, which are supported by a robust collection of Manuals & Tutorials to get you started with RealDash.

Another great feature of WiCAN-OBD is its MQTT battery alerts. It can monitor your battery voltage and send an alert if that voltage drops under a set threshold. This feature is especially important for users who own multiple cars they do not use regularly.

## WiCAN-OBD2 Pinout

<p align="center">
<img src="https://user-images.githubusercontent.com/94690098/182854687-911bae04-9bdd-4947-8363-e088e278b3b8.png" >
</p>

## [**Programming Examples**](https://github.com/meatpiHQ/programming_examples/tree/master/CAN)

### **Features and Specifications**:

- Supports CAN2.0A/B up to 1Mbits.
- Works with Realdash, based on "realdash 66"
- Supports SocketCAN and works with BUSMaster
- Supports TCP and UDP
- WiFi can be used in AP and station mode
- WiFi and CAN configured using web interface.
- Diode protection for the USB port

![Modes](https://user-images.githubusercontent.com/94690098/222961571-bd137341-808a-4f0a-9528-789fe24d640e.png "Connection Mode")

---

Images of WiCANs © 2026 meatPi Electronics | www.meatpi.com | PO Box 5005 Clayton, VIC 3168, Australia
