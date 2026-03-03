/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * usb_descriptors.c — TinyUSB CDC device/configuration/string descriptors.
 *
 * VID 0x2E8A = Raspberry Pi Foundation
 * PID 0x1000 = placeholder (no official assignment; change when registered)
 *
 * Configuration descriptor has bmAttributes=0xA0 (bus-powered + remote wakeup)
 * to enable hardware wakeup from PC suspend.
 *
 * Interface layout:
 *   Interface 0: CDC Communication (notification endpoint 0x81)
 *   Interface 1: CDC Data          (bulk OUT 0x02, bulk IN 0x82)
 */

#include "tusb.h"

/* ------------------------------------------------------------------ */
/* Device descriptor                                                    */
/* ------------------------------------------------------------------ */

tusb_desc_device_t const desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,

    /* Use Miscellaneous Device Class + IAD so CDC interfaces are grouped. */
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,

    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0x2E8A,  /* Raspberry Pi */
    .idProduct          = 0x1000,  /* placeholder */
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01,
};

uint8_t const *tud_descriptor_device_cb(void) {
    return (uint8_t const *)&desc_device;
}

/* ------------------------------------------------------------------ */
/* Configuration descriptor                                             */
/* ------------------------------------------------------------------ */

enum {
    ITF_NUM_CDC = 0,
    ITF_NUM_CDC_DATA,
    ITF_NUM_TOTAL,
};

#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_CDC_DESC_LEN)

/* Endpoints */
#define EP_CDC_NOTIF   0x81
#define EP_CDC_OUT     0x02
#define EP_CDC_IN      0x82

uint8_t const desc_fs_configuration[] = {
    /* Configuration descriptor
     * bmAttributes = 0xA0: bit7 (reserved=1) | bit5 (remote wakeup)
     * bMaxPower = 50 → 100 mA */
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0xA0, 50),

    /* CDC descriptor: comm interface, notification EP, data interface, bulk EPs */
    TUD_CDC_DESCRIPTOR(ITF_NUM_CDC, 4, EP_CDC_NOTIF, 8, EP_CDC_OUT, EP_CDC_IN, 64),
};

uint8_t const *tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return desc_fs_configuration;
}

/* ------------------------------------------------------------------ */
/* String descriptors                                                   */
/* ------------------------------------------------------------------ */

/* String index 0 = language table.  0x0409 = US English. */
static uint16_t const s_lang_table[] = {
    (TUSB_DESC_STRING << 8) | (2 * sizeof(uint16_t)),
    0x0409,
};

/* String table (indices 1..): manufacturer, product, serial, CDC interface. */
static char const *const s_string_table[] = {
    "impulse-cec",            /* 1: Manufacturer */
    "Pulse-Eight CEC Adapter",/* 2: Product */
    "000001",                 /* 3: Serial number */
    "CDC Data",               /* 4: CDC interface name */
};

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;

    static uint16_t s_desc_str[32];
    uint8_t chr_count;

    if (index == 0) {
        return s_lang_table;
    }

    /* Indices 1.. map to s_string_table[index-1]. */
    if (index > (uint8_t)(sizeof(s_string_table) / sizeof(s_string_table[0]))) {
        return NULL;
    }

    const char *str = s_string_table[index - 1];
    chr_count = (uint8_t)strlen(str);
    if (chr_count > 31) chr_count = 31;

    /* First word: length (bytes) + string descriptor type */
    s_desc_str[0] = (TUSB_DESC_STRING << 8) | (2u + 2u * chr_count);

    /* Convert ASCII to UTF-16. */
    for (uint8_t i = 0; i < chr_count; i++) {
        s_desc_str[1 + i] = (uint16_t)str[i];
    }

    return s_desc_str;
}
