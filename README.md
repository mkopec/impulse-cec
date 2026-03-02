# Pulse-Eight CEC Dongle

A Pulse-Eight compatible USB CEC adapter that speaks the P8 binary protocol, compatible with both **libCEC** (`cec-client`) and the **Linux kernel `pulse8-cec` driver** (`inputattach`).

Supports two boards:

| | Seeed XIAO ESP32-C6 | Seeed XIAO RP2350 |
|---|---|---|
| CEC GPIO | GPIO21 (pad D3) | GPIO29 (pad D3) |
| USB | Serial/JTAG — fixed VID:PID `0x303A:0x1001` | TinyUSB — `0x2E8A:0x1000` |
| libCEC auto-detect | No (specify port with `-t p`) | No (PID not yet registered) |
| Settings storage | NVS | Flash (last sector) |
| USB remote wakeup | No | Yes |
| Build system | ESP-IDF v6.x | Pico SDK |

## Hardware

**Wiring (both boards — same physical pin):**

| XIAO pad | ESP32-C6 GPIO | RP2350 GPIO | Connect to   |
|----------|---------------|-------------|--------------|
| D3       | GPIO21        | GPIO29      | HDMI CEC pin |
| GND      | —             | —           | HDMI GND     |

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

**RP2350:** requires the [Pico SDK](https://github.com/raspberrypi/pico-sdk) and a CMake ≥ 3.13 + ARM toolchain. Set `PICO_SDK_PATH` in the environment or on the `make` command line.

## Flashing

**ESP32-C6:**

```bash
make flash-esp32 PORT=/dev/ttyACM0
```

**RP2350:**

```bash
make flash-rp2350       # uses picotool if available
```

Or manually: hold BOOTSEL, plug in the board, then copy `rp2350/build/esp32-p8-rp2350.uf2` to the `RPI-RP2` drive.

## Usage

### libCEC / cec-client

```bash
cec-client -t p /dev/ttyACM0
```

The `-t p` flag selects Pulse-Eight adapter type. Specify the port explicitly — neither board's VID:PID is registered for libCEC auto-detection yet. On macOS use `/dev/cu.usbmodem*`; on Windows use `COMx`.

### Linux kernel driver (inputattach)

The `pulse8-cec` driver exposes the adapter as a standard Linux CEC device. TV remote presses generate input events; CEC commands can be sent with `cec-ctl`.

#### One-time system setup

```bash
echo "options pulse8-cec persistent_config=1" | sudo tee /etc/modprobe.d/pulse8-cec.conf
sudo modprobe -r pulse8-cec && sudo modprobe pulse8-cec
```

`persistent_config=1` is required for the driver to automatically claim a logical address on startup.

#### Attach the adapter

```bash
sudo inputattach -p8 /dev/ttyACM0
```

The driver sets PA=1.0.0.0, claims logical address 4 (Playback Device), and enables the ACK mask.

#### Alternative: udev rule (no modprobe change needed)

```
# /etc/udev/rules.d/99-cec-pulse8-pa.rules
SUBSYSTEM=="cec", ACTION=="add", RUN+="/usr/bin/cec-ctl -d /dev/%k -p 1.0.0.0"
```

## Architecture

Both targets implement the same P8 protocol logic (`shared/p8_protocol.c`) over a platform abstraction layer (`shared/platform.h`). The CEC driver is platform-specific.

```
USB (CDC ACM)
      │
      ▼
p8_protocol.c      shared — Pulse-Eight binary protocol parser/encoder
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
esp32-p8/
├── Makefile
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
