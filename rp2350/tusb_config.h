/*
 * tusb_config.h — TinyUSB configuration for esp32-p8 RP2350 build.
 *
 * CFG_TUSB_MCU is injected by the Pico SDK CMake integration.
 * We only need to configure the device-side CDC class here.
 */

#pragma once

/* ---- Controller and OS ---- */
#define CFG_TUSB_RHPORT0_MODE    OPT_MODE_DEVICE
#define CFG_TUSB_OS              OPT_OS_NONE   /* bare-metal, no RTOS */

/* ---- Device-side classes ---- */
#define CFG_TUD_CDC              1    /* one CDC ACM interface */
#define CFG_TUD_MSC              0
#define CFG_TUD_HID              0
#define CFG_TUD_MIDI             0
#define CFG_TUD_VENDOR           0

/* ---- CDC buffer sizes ---- */
#define CFG_TUD_CDC_RX_BUFSIZE   256
#define CFG_TUD_CDC_TX_BUFSIZE   256
#define CFG_TUD_CDC_EP_BUFSIZE   64

/* ---- Control endpoint size ---- */
#define CFG_TUD_ENDPOINT0_SIZE   64
