# SPDX-License-Identifier: GPL-3.0-or-later

idf_component_register(
    SRCS
        "main.c"
        "cec_bus.c"
        "../../shared/p8_protocol.c"
    INCLUDE_DIRS
        "."
        "../../shared"
    REQUIRES
        esp_driver_gpio
        esp_driver_usb_serial_jtag
        esp_timer
        freertos
        nvs_flash
)
