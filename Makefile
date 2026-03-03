# SPDX-License-Identifier: GPL-3.0-or-later

# Pulse-Eight CEC Dongle — top-level build wrapper
#
# Prerequisites
# =============
#   ESP32-C6:  ESP-IDF v6.x sourced in the shell
#                . /path/to/esp-idf/export.sh
#   RP2350:    PICO_SDK_PATH set (env or make arg)
#                make rp2350 PICO_SDK_PATH=~/pico-sdk
#
# Common targets
# ==============
#   make esp32             build ESP32-C6 firmware
#   make rp2350            configure + build RP2350 firmware
#   make flash-esp32       flash ESP32-C6 over USB (PORT=...)
#   make flash-rp2350      flash RP2350 (picotool if available, else prints path)
#   make monitor-esp32     open serial monitor
#   make clean             remove both build trees
#   make help              list all targets

PORT          ?= /dev/ttyACM0
PICO_SDK_PATH ?= $(HOME)/Development/pico-sdk
PICO_PLATFORM ?= rp2350-arm-s

RP2350_BUILD  := rp2350/build
RP2350_UF2    := $(RP2350_BUILD)/impulse-cec.uf2

# ------------------------------------------------------------------ #
# Default: show help                                                  #
# ------------------------------------------------------------------ #

.DEFAULT_GOAL := help

.PHONY: help
help:
	@echo "Pulse-Eight CEC Dongle build targets:"
	@echo ""
	@echo "  make esp32              build ESP32-C6 firmware"
	@echo "  make rp2350             build RP2350 firmware"
	@echo "  make all                build both"
	@echo ""
	@echo "  make flash-esp32        flash ESP32-C6  (PORT=$(PORT))"
	@echo "  make flash-rp2350       flash RP2350 via picotool, or print .uf2 path"
	@echo "  make monitor-esp32      open serial monitor (PORT=$(PORT))"
	@echo ""
	@echo "  make clean-esp32        remove esp32/build"
	@echo "  make clean-rp2350       remove rp2350/build"
	@echo "  make clean              remove both build trees"
	@echo ""
	@echo "Variables (override on command line or in environment):"
	@echo "  PORT=$(PORT)"
	@echo "  PICO_SDK_PATH=$(PICO_SDK_PATH)"
	@echo "  PICO_PLATFORM=$(PICO_PLATFORM)"

# ------------------------------------------------------------------ #
# ESP32-C6 (ESP-IDF)                                                  #
# ------------------------------------------------------------------ #

.PHONY: esp32 flash-esp32 monitor-esp32 clean-esp32

esp32:
	@command -v idf.py >/dev/null 2>&1 || { \
	  echo ""; \
	  echo "Error: idf.py not found."; \
	  echo "Source the ESP-IDF environment first:"; \
	  echo "  . /path/to/esp-idf/export.sh"; \
	  echo ""; \
	  exit 1; \
	}
	cd esp32 && idf.py build

flash-esp32:
	cd esp32 && idf.py -p $(PORT) flash

monitor-esp32:
	cd esp32 && idf.py -p $(PORT) monitor

flash-monitor-esp32: flash-esp32
	cd esp32 && idf.py -p $(PORT) monitor

clean-esp32:
	rm -rf esp32/build

# ------------------------------------------------------------------ #
# RP2350 (Pico SDK)                                                   #
# ------------------------------------------------------------------ #

.PHONY: rp2350 flash-rp2350 clean-rp2350

$(RP2350_BUILD)/Makefile:
	@test -d "$(PICO_SDK_PATH)" || { \
	  echo ""; \
	  echo "Error: PICO_SDK_PATH=$(PICO_SDK_PATH) does not exist."; \
	  echo "Set it via:  make rp2350 PICO_SDK_PATH=/path/to/pico-sdk"; \
	  echo ""; \
	  exit 1; \
	}
	cmake -S rp2350 -B $(RP2350_BUILD) \
	  -DPICO_SDK_PATH=$(PICO_SDK_PATH) \
	  -DPICO_PLATFORM=$(PICO_PLATFORM)

rp2350: $(RP2350_BUILD)/Makefile
	cmake --build $(RP2350_BUILD)

flash-rp2350: rp2350
	@if command -v picotool >/dev/null 2>&1; then \
	  echo "Flashing via picotool (hold BOOTSEL or the device must be in BOOTROM mode)..."; \
	  picotool load -f $(RP2350_UF2); \
	else \
	  echo ""; \
	  echo "picotool not found. Copy the UF2 manually:"; \
	  echo "  1. Hold BOOTSEL and plug in the XIAO RP2350"; \
	  echo "  2. Copy $(RP2350_UF2) to the RPI-RP2 drive"; \
	  echo ""; \
	fi

clean-rp2350:
	rm -rf $(RP2350_BUILD)

# ------------------------------------------------------------------ #
# Combined                                                            #
# ------------------------------------------------------------------ #

.PHONY: all clean

all: esp32 rp2350

clean: clean-esp32 clean-rp2350
