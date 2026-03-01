# ESP32-C6 Pulse-Eight CEC Dongle

A Pulse-Eight compatible USB CEC adapter running on the Seeed XIAO ESP32-C6. It presents over USB as a standard serial port and speaks the P8 binary protocol, making it compatible with both **libCEC** (`cec-client`) and the **Linux kernel `pulse8-cec` driver** (`inputattach`).

## Hardware

**Board:** Seeed XIAO ESP32-C6

**Wiring:**

| XIAO pin | GPIO   | Connect to       |
|----------|--------|------------------|
| D3       | GPIO21 | HDMI CEC pin     |
| GND      | —      | HDMI GND         |

The firmware enables the ESP32-C6 internal ~47kΩ pull-up on the CEC line. No external pull-up is required for direct connections or short cables. For long cable runs, an optional external 27kΩ resistor from CEC to 3.3V (paralleling to ~18kΩ) improves signal integrity.

**USB note:** The ESP32-C6 has a USB Serial/JTAG peripheral (not full USB OTG), so it appears with a fixed Espressif VID:PID (`0x303A:0x1001`). libCEC won't auto-detect it as Pulse-Eight — specify the port explicitly (see [Usage](#usage)).

## Building and Flashing

Requires [ESP-IDF v6.1.0](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c6/).

```bash
# One-time: source the IDF environment (required each new shell session)
. $IDF_PATH/export.sh

# One-time: set the target
idf.py set-target esp32c6

# Build, flash, and open serial monitor
idf.py build
idf.py -p /dev/ttyACM0 flash monitor
```

## Usage

### libCEC / cec-client

```bash
cec-client -t p /dev/ttyACM0
```

The `-t p` flag selects Pulse-Eight adapter type. On macOS use `/dev/cu.usbmodem*`; on Windows use `COMx`.

### Linux kernel driver (inputattach)

The kernel `pulse8-cec` driver enables TV remote button presses to generate standard Linux input events and allows sending CEC commands directly from the kernel CEC subsystem.

#### One-time system setup

```bash
echo "options pulse8-cec persistent_config=1" | sudo tee /etc/modprobe.d/pulse8-cec.conf
sudo modprobe -r pulse8-cec && sudo modprobe pulse8-cec
```

The `persistent_config=1` parameter is required for the driver to automatically claim a logical address on startup.

#### Attach the adapter

```bash
sudo inputattach -p8 /dev/ttyACM0
```

The driver will set PA=1.0.0.0, claim logical address 4 (Playback Device), and enable the ACK mask. TV remote presses will appear as Linux input events; CEC commands (e.g. standby) can be sent via `cec-ctl`.

#### Alternative: udev rule (no modprobe change needed)

```
# /etc/udev/rules.d/99-cec-pulse8-pa.rules
SUBSYSTEM=="cec", ACTION=="add", RUN+="/usr/bin/cec-ctl -d /dev/%k -p 1.0.0.0"
```

## Architecture

```
USB Serial/JTAG (CDC ACM)
        │
        ▼
  p8_protocol.c        — Pulse-Eight binary protocol parser/encoder
        │
        ▼
   cec_bus.c           — Open-drain bit-bang CEC driver on GPIO21
        │
        ▼
  HDMI CEC line
```

**CEC RX:** GPIO ANYEDGE ISR → edge-event queue → high-priority decoder task

**CEC TX:** `esp_timer` callbacks chain through waveform phases (no busy-waiting). ACK generation is driven from ISR context for accurate timing.

**Settings** are persisted in NVS (`p8` namespace) and survive power cycles.

## Project Structure

```
esp32-p8/
├── CMakeLists.txt
├── sdkconfig.defaults        — USB CDC config, 1kHz FreeRTOS tick, ISR timer
└── main/
    ├── CMakeLists.txt
    ├── cec_bus.h / cec_bus.c  — GPIO open-drain CEC bit-bang driver
    ├── p8_protocol.h / .c     — Pulse-Eight serial protocol handler
    └── main.c                 — Initialisation and USB RX task
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
