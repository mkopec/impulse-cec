/*
 * flash_kv.h — simple key-value settings store backed by one flash sector.
 *
 * Settings are kept in a 256-byte struct at the last flash sector.
 * A magic word identifies valid data; if absent, defaults are used.
 *
 * All get/set operations work on a RAM copy.  Call flash_kv_commit() to
 * persist changes (performs a sector erase + page program with interrupts
 * disabled).
 *
 * Keys mirror the NVS keys used by the ESP32-C6 build:
 *   "auto_en"    uint8  — auto_enabled
 *   "auto_pwr"   uint8  — auto_power_on
 *   "def_la"     uint8  — default_logical_addr
 *   "dev_type"   uint8  — device_type
 *   "hdmi_ver"   uint8  — hdmi_version
 *   "la_mask"    uint16 — logical_addr_mask
 *   "phys_addr"  uint16 — physical_addr
 *   "osd_name"   str    — OSD name (max 13 chars + NUL)
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* Magic value stored in flash to identify valid settings. */
#define FLASH_SETTINGS_MAGIC  0x50383552u  /* "P85R" */

/* Settings struct — must fit in one 256-byte flash page. */
typedef struct __attribute__((packed)) {
    uint32_t magic;
    uint8_t  auto_enabled;
    uint8_t  auto_power_on;
    uint8_t  default_la;
    uint8_t  device_type;
    uint8_t  hdmi_version;
    uint8_t  _pad1;
    uint16_t logical_addr_mask;
    uint16_t physical_addr;
    char     osd_name[14];
    uint8_t  _pad2[256 - 24];
} flash_settings_t;

/* Initialise: load from flash if magic matches, otherwise use defaults. */
void flash_kv_init(void);

/* Read a uint8 setting by key; returns def if key unknown. */
uint8_t  flash_kv_get_u8 (const char *key, uint8_t  def);

/* Read a uint16 setting by key; returns def if key unknown. */
uint16_t flash_kv_get_u16(const char *key, uint16_t def);

/* Read a string setting into buf[0..n-1]; returns true if found. */
bool     flash_kv_get_str(const char *key, char *buf, size_t n);

/* Write a uint8 setting (RAM only until flash_kv_commit). */
void flash_kv_set_u8 (const char *key, uint8_t  val);

/* Write a uint16 setting (RAM only until flash_kv_commit). */
void flash_kv_set_u16(const char *key, uint16_t val);

/* Write a string setting (RAM only until flash_kv_commit). */
void flash_kv_set_str(const char *key, const char *val);

/* Erase + program the flash sector with current RAM settings. */
void flash_kv_commit(void);
