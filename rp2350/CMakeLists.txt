cmake_minimum_required(VERSION 3.13)

# Point to the Pico SDK.  Override via environment or -DPICO_SDK_PATH=...
if(DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}")
endif()
if(NOT PICO_SDK_PATH)
    message(FATAL_ERROR "PICO_SDK_PATH is not set. "
        "Export it or pass -DPICO_SDK_PATH=/path/to/pico-sdk")
endif()

include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(esp32-p8-rp2350 C CXX ASM)
set(PICO_BOARD seeed_xiao_rp2350)
pico_sdk_init()

add_executable(esp32-p8-rp2350
    main.c
    cec_bus.c
    flash_kv.c
    usb_descriptors.c
    ../shared/p8_protocol.c
)

target_include_directories(esp32-p8-rp2350 PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}           # rp2350/: flash_kv.h, usb_descriptors
    ${CMAKE_CURRENT_LIST_DIR}/../shared  # cec_bus.h, p8_protocol.h, platform.h
)

target_link_libraries(esp32-p8-rp2350
    pico_stdlib
    hardware_gpio
    hardware_timer
    hardware_flash
    hardware_sync
    tinyusb_device
    tinyusb_board
)

# TinyUSB config header must be on the include path.
# Create a minimal tusb_config.h if not already present.
target_compile_definitions(esp32-p8-rp2350 PRIVATE
    # Nothing extra needed; TinyUSB picks up tusb_config.h from include path.
)

# Disable SDK stdio — we own the USB CDC port entirely via TinyUSB.
pico_enable_stdio_usb(esp32-p8-rp2350 0)
pico_enable_stdio_uart(esp32-p8-rp2350 0)

# Generate UF2 and other flash images.
pico_add_extra_outputs(esp32-p8-rp2350)
