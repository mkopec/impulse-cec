<!--
SPDX-License-Identifier: GPL-3.0-or-later
-->

# Impulse CEC

A USB CEC dongle for the Seeed XIAO ESP32-C6 and XIAO RP2350. It bit-bangs the HDMI CEC bus and exposes it over USB serial using the open P8 adapter protocol, making it work out of the box with **libCEC** (`cec-client`) and the **Linux kernel `pulse8-cec` driver** (`inputattach`).

> **Note:** This project is not affiliated with or endorsed by Pulse-Eight Ltd. "Pulse-Eight" is a trademark of Pulse-Eight Ltd. The P8 serial protocol is an open, documented wire format independently implemented here.

Supports two boards:

| | Seeed XIAO ESP32-C6 | Seeed XIAO RP2350 |
|---|---|---|
| CEC GPIO | GPIO21 (pad D3) | GPIO5 (pad D3) |
| USB | Serial/JTAG — fixed VID:PID `0x303A:0x1001` | TinyUSB — `0x2E8A:0x1000` |
| libCEC auto-detect | No (specify port with `-t p`) | No (PID not yet registered) |
| Settings storage | NVS | Flash (last sector) |
| USB remote wakeup | No | Yes |
| Build system | ESP-IDF v6.x | Pico SDK |

## Hardware

**Wiring (both boards — same physical pin):**

| XIAO pad | ESP32-C6 GPIO | RP2350 GPIO | Connect to   |
|----------|---------------|-------------|--------------|
| D3       | GPIO21        | GPIO5       | HDMI CEC pin |
| GND      | —             | —           | HDMI CEC GND |

The firmware enables an internal pull-up on the CEC line (~47kΩ on ESP32-C6, configurable on RP2350). No external pull-up is required for direct connections or short cables. For long cable runs, an optional 27kΩ resistor from CEC to 3.3V (paralleling to ~18kΩ) improves signal integrity.

## Building

```bash
# ESP32-C6 (source ESP-IDF first)
. /path/to/esp-idf/export.sh
make esp32

# RP2350
make rp2350 PICO_SDK_PATH=~/pico-sdk

# Both
make all PICO_SDK_PATH=~/pico-sdk
```

Run `make help` for all available targets (flash, monitor, clean, …).

### Toolchain setup

**ESP32-C6:** requires [ESP-IDF v6.x](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/). Source `export.sh` in each new shell before building.

**RP2350:** requires the [Pico SDK](https://github.com/raspberrypi/pico-sdk) and CMake ≥ 3.13 + an ARM toolchain. Set `PICO_SDK_PATH` in the environment or on the `make` command line.

## Flashing

**ESP32-C6:**

```bash
make flash-esp32 PORT=/dev/ttyACM0
```

**RP2350:**

```bash
make flash-rp2350       # uses picotool if available
```

Or manually: hold BOOTSEL, plug in the board, then copy `rp2350/build/impulse-cec.uf2` to the `RPI-RP2` drive.

## Usage

### libCEC / cec-client

```bash
cec-client -t p /dev/ttyACM0
```

The `-t p` flag selects the P8 adapter protocol. Specify the port explicitly — neither board's VID:PID triggers libCEC auto-detection yet. On macOS use `/dev/cu.usbmodem*`; on Windows use `COMx`.

### Linux kernel driver (inputattach)

The kernel `pulse8-cec` driver speaks the same P8 protocol and exposes the dongle as a standard Linux CEC device. TV remote button presses generate input events; CEC commands can be sent with `cec-ctl`.

#### One-time system setup

```bash
echo "options pulse8-cec persistent_config=1" | sudo tee /etc/modprobe.d/pulse8-cec.conf
sudo modprobe -r pulse8-cec && sudo modprobe pulse8-cec
```

`persistent_config=1` is required for the driver to automatically claim a logical address on startup.

#### Attach the dongle

```bash
sudo inputattach -p8 /dev/ttyACM0
```

The driver sets PA=1.0.0.0, claims logical address 4 (Playback Device), and enables the ACK mask.

#### Alternative: udev rule (no modprobe change needed)

```
# /etc/udev/rules.d/99-cec-pa.rules
SUBSYSTEM=="cec", ACTION=="add", RUN+="/usr/bin/cec-ctl -d /dev/%k -p 1.0.0.0"
```

### Wake-on-CEC (RP2350 only)

The RP2350 firmware can wake the PC from suspend when it receives a CEC power-on command or a `Set Stream Path` addressed to the dongle's physical address. For this to work, Linux must enable USB remote wakeup for the device.

Install the included udev rule to enable it automatically:

```bash
sudo cp udev/99-impulse-cec.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Verify it took effect:

```bash
cat /sys/bus/usb/devices/$(ls /sys/bus/usb/drivers/usb/ | \
  xargs -I{} sh -c 'cat /sys/bus/usb/drivers/usb/{}/idVendor 2>/dev/null | grep -q 2e8a && echo {}')/power/wakeup
# should print: enabled
```

## Architecture

Both targets share the same protocol implementation (`shared/p8_protocol.c`) through a thin platform abstraction layer (`shared/platform.h`). The CEC bus driver is platform-specific.

```
USB (CDC ACM)
      │
      ▼
p8_protocol.c      shared — P8 serial protocol parser/encoder
      │
      ▼
cec_bus.c          platform-specific — open-drain bit-bang CEC driver
      │
      ▼
HDMI CEC line
```

**ESP32-C6:** GPIO ANYEDGE ISR → edge-event queue → FreeRTOS decoder task. TX uses `esp_timer` ISR callbacks to chain waveform phases without busy-waiting.

**RP2350:** GPIO IRQ stores edges in a ring buffer. Main loop drains it via `cec_rx_process()`. TX uses hardware alarms for phase transitions. TX results are deferred to the main loop so TinyUSB writes never happen from IRQ context.

## Project Structure

```
impulse-cec/
├── Makefile
├── udev/
│   └── 99-impulse-cec.rules  — enables USB remote wakeup on Linux
├── shared/                 — portable sources (both targets)
│   ├── cec_bus.h           — CEC driver interface + portability typedefs
│   ├── platform.h          — platform abstraction (USB write, KV store, mutex)
│   ├── p8_protocol.h
│   └── p8_protocol.c
├── esp32/                  — ESP-IDF project (cd here for idf.py)
│   ├── CMakeLists.txt
│   ├── sdkconfig.defaults
│   └── main/
│       ├── CMakeLists.txt
│       ├── cec_bus.c       — ESP32-C6 CEC driver (GPIO ISR + FreeRTOS tasks)
│       └── main.c          — app_main: NVS init, USB Serial/JTAG, tasks
└── rp2350/                 — Pico SDK project (cd here for cmake)
    ├── CMakeLists.txt
    ├── tusb_config.h
    ├── cec_bus.c           — RP2350 CEC driver (GPIO IRQ + hardware alarms)
    ├── main.c              — main loop: tud_task + cec_rx_process + cec_bus_tick
    ├── flash_kv.h / .c     — settings in last flash sector
    └── usb_descriptors.c   — TinyUSB CDC descriptors (VID=0x2E8A PID=0x1000)
```

## CEC Timing

| Symbol        | Value   |
|---------------|---------|
| Start LOW     | 3700 µs |
| Start HIGH    | 800 µs  |
| Bit '0' LOW   | 1500 µs |
| Bit '1' LOW   | 600 µs  |
| Bit period    | 2400 µs |
| Sample point  | 1050 µs from falling edge |
