/*
 * flash_kv.c — key-value settings store backed by one flash sector.
 *
 * Layout: last sector of flash (offset = PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE).
 * The struct fits in one 256-byte page, so we erase the full sector and
 * program exactly one page on each commit.
 *
 * Read path: XIP memory-mapped access (no copy needed, but we take a RAM
 * copy on init for easy read-back without pointer aliasing concerns).
 *
 * Write path (flash_kv_commit):
 *   1. Disable interrupts (required while executing flash operations)
 *   2. flash_range_erase   — erases the entire sector (4096 bytes)
 *   3. flash_range_program — programs the first 256-byte page
 *   4. Restore interrupts
 *
 * NOTE: flash operations must NOT be called while code is executing from
 * flash on the same core.  On RP2350 in single-core mode this is safe as
 * long as interrupts are disabled (no ISR can trigger a flash access).
 */

#include "flash_kv.h"

#include <string.h>
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/platform.h"

/* Flash offset of the settings sector (last sector). */
#define SETTINGS_OFFSET  (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

/* XIP base address for memory-mapped read. */
#ifndef XIP_BASE
#define XIP_BASE  0x10000000u
#endif

/* RAM copy of settings — all operations work on this. */
static flash_settings_t s_settings;
static bool s_dirty = false;

/* ------------------------------------------------------------------ */
/* Init                                                                 */
/* ------------------------------------------------------------------ */

void flash_kv_init(void) {
    const flash_settings_t *flash_ptr =
        (const flash_settings_t *)(uintptr_t)(XIP_BASE + SETTINGS_OFFSET);

    if (flash_ptr->magic == FLASH_SETTINGS_MAGIC) {
        s_settings = *flash_ptr;
    } else {
        /* First boot or corrupted flash — set defaults. */
        memset(&s_settings, 0xFF, sizeof(s_settings));
        s_settings.magic             = FLASH_SETTINGS_MAGIC;
        s_settings.auto_enabled      = 1;
        s_settings.auto_power_on     = 0;
        s_settings.default_la        = 0x0F;
        s_settings.device_type       = 4;    /* Playback */
        s_settings.hdmi_version      = 0x05;
        s_settings._pad1             = 0xFF;
        s_settings.logical_addr_mask = 0x0000;
        s_settings.physical_addr     = 0x1000; /* 1.0.0.0 */
        strlcpy(s_settings.osd_name, "ESP32-P8", sizeof(s_settings.osd_name));
        memset(s_settings._pad2, 0xFF, sizeof(s_settings._pad2));
    }
    s_dirty = false;
}

/* ------------------------------------------------------------------ */
/* Get                                                                  */
/* ------------------------------------------------------------------ */

uint8_t flash_kv_get_u8(const char *key, uint8_t def) {
    if (__builtin_strcmp(key, "auto_en")  == 0) return s_settings.auto_enabled;
    if (__builtin_strcmp(key, "auto_pwr") == 0) return s_settings.auto_power_on;
    if (__builtin_strcmp(key, "def_la")   == 0) return s_settings.default_la;
    if (__builtin_strcmp(key, "dev_type") == 0) return s_settings.device_type;
    if (__builtin_strcmp(key, "hdmi_ver") == 0) return s_settings.hdmi_version;
    return def;
}

uint16_t flash_kv_get_u16(const char *key, uint16_t def) {
    if (__builtin_strcmp(key, "la_mask")   == 0) return s_settings.logical_addr_mask;
    if (__builtin_strcmp(key, "phys_addr") == 0) return s_settings.physical_addr;
    return def;
}

bool flash_kv_get_str(const char *key, char *buf, size_t n) {
    if (__builtin_strcmp(key, "osd_name") == 0) {
        strlcpy(buf, s_settings.osd_name, n);
        return true;
    }
    return false;
}

/* ------------------------------------------------------------------ */
/* Set                                                                  */
/* ------------------------------------------------------------------ */

void flash_kv_set_u8(const char *key, uint8_t val) {
    if      (__builtin_strcmp(key, "auto_en")  == 0) s_settings.auto_enabled  = val;
    else if (__builtin_strcmp(key, "auto_pwr") == 0) s_settings.auto_power_on = val;
    else if (__builtin_strcmp(key, "def_la")   == 0) s_settings.default_la    = val;
    else if (__builtin_strcmp(key, "dev_type") == 0) s_settings.device_type   = val;
    else if (__builtin_strcmp(key, "hdmi_ver") == 0) s_settings.hdmi_version  = val;
    else return;
    s_dirty = true;
}

void flash_kv_set_u16(const char *key, uint16_t val) {
    if      (__builtin_strcmp(key, "la_mask")   == 0) s_settings.logical_addr_mask = val;
    else if (__builtin_strcmp(key, "phys_addr") == 0) s_settings.physical_addr     = val;
    else return;
    s_dirty = true;
}

void flash_kv_set_str(const char *key, const char *val) {
    if (__builtin_strcmp(key, "osd_name") == 0) {
        strlcpy(s_settings.osd_name, val, sizeof(s_settings.osd_name));
        s_dirty = true;
    }
}

/* ------------------------------------------------------------------ */
/* Commit                                                               */
/* ------------------------------------------------------------------ */

void flash_kv_commit(void) {
    if (!s_dirty) return;

    /* flash_range_program requires the data buffer to be in RAM (not XIP). */
    static uint8_t page_buf[FLASH_PAGE_SIZE];
    memset(page_buf, 0xFF, sizeof(page_buf));
    memcpy(page_buf, &s_settings, sizeof(s_settings));

    uint32_t saved = save_and_disable_interrupts();
    flash_range_erase(SETTINGS_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(SETTINGS_OFFSET, page_buf, FLASH_PAGE_SIZE);
    restore_interrupts(saved);

    s_dirty = false;
}
