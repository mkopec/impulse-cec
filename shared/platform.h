/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * platform.h — thin portability layer for p8_protocol.c
 *
 * Provides:
 *   platform_init()                   — one-time init (opens NVS on ESP32-C6)
 *   platform_usb_write(buf, len)      — write + flush USB CDC
 *   platform_kv_get_u8/u16/str()      — persistent key-value read
 *   platform_kv_set_u8/u16/str()      — persistent key-value write
 *   platform_kv_commit()              — flush writes to storage
 *   p8_mutex_t / platform_mutex_*()   — serialise USB writes (no-op on RP2350)
 *   platform_sleep_ms(ms)             — delay
 *   platform_restart()                — reboot device
 *   PLAT_LOGI/W/D(tag, fmt, ...)      — logging
 *
 * On ESP32-C6 (#ifdef IDF_VER):  NVS, usb_serial_jtag, FreeRTOS semphr, esp_log
 * On RP2350 (else):               flash_kv, TinyUSB CDC, single-core no-op mutex
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* ================================================================== */
/* ESP32-C6 / ESP-IDF                                                   */
/* ================================================================== */
#ifdef IDF_VER

#include <string.h>
#include "esp_log.h"
#include "esp_system.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/usb_serial_jtag.h"

/* ---- NVS state (file-scope, only platform.h users see this) ---- */
static nvs_handle_t s_p8_nvs;
static bool         s_p8_nvs_ok = false;

static inline void platform_init(void) {
    esp_err_t ret = nvs_open("p8", NVS_READWRITE, &s_p8_nvs);
    if (ret == ESP_OK) {
        s_p8_nvs_ok = true;
    } else {
        ESP_LOGW("platform", "NVS open failed (%s), using defaults",
                 esp_err_to_name(ret));
    }
}

/* ---- USB write ---- */
static inline void platform_usb_write(const uint8_t *buf, size_t len) {
    usb_serial_jtag_write_bytes(buf, len, pdMS_TO_TICKS(50));
    usb_serial_jtag_wait_tx_done(pdMS_TO_TICKS(50));
}

/* ---- KV: get ---- */
static inline uint8_t platform_kv_get_u8(const char *key, uint8_t def) {
    uint8_t val = def;
    if (s_p8_nvs_ok) nvs_get_u8(s_p8_nvs, key, &val);
    return val;
}
static inline uint16_t platform_kv_get_u16(const char *key, uint16_t def) {
    uint16_t val = def;
    if (s_p8_nvs_ok) nvs_get_u16(s_p8_nvs, key, &val);
    return val;
}
static inline bool platform_kv_get_str(const char *key, char *buf, size_t n) {
    if (!s_p8_nvs_ok) return false;
    size_t sz = n;
    return nvs_get_str(s_p8_nvs, key, buf, &sz) == ESP_OK;
}

/* ---- KV: set ---- */
static inline void platform_kv_set_u8(const char *key, uint8_t val) {
    if (s_p8_nvs_ok) nvs_set_u8(s_p8_nvs, key, val);
}
static inline void platform_kv_set_u16(const char *key, uint16_t val) {
    if (s_p8_nvs_ok) nvs_set_u16(s_p8_nvs, key, val);
}
static inline void platform_kv_set_str(const char *key, const char *val) {
    if (s_p8_nvs_ok) nvs_set_str(s_p8_nvs, key, val);
}
static inline void platform_kv_commit(void) {
    if (s_p8_nvs_ok) nvs_commit(s_p8_nvs);
}

/* ---- Mutex ---- */
typedef struct { SemaphoreHandle_t h; } p8_mutex_t;

static inline void platform_mutex_init(p8_mutex_t *m) {
    m->h = xSemaphoreCreateMutex();
}
static inline void platform_mutex_lock(p8_mutex_t *m) {
    xSemaphoreTake(m->h, portMAX_DELAY);
}
static inline void platform_mutex_unlock(p8_mutex_t *m) {
    xSemaphoreGive(m->h);
}

/* ---- System ---- */
static inline void platform_sleep_ms(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}
static inline void platform_restart(void) {
    esp_restart();
}

/* ---- Logging ---- */
#define PLAT_LOGI(tag, fmt, ...)  ESP_LOGI(tag, fmt, ##__VA_ARGS__)
#define PLAT_LOGW(tag, fmt, ...)  ESP_LOGW(tag, fmt, ##__VA_ARGS__)
#define PLAT_LOGD(tag, fmt, ...)  ESP_LOGD(tag, fmt, ##__VA_ARGS__)

/* ================================================================== */
/* RP2350 / Pico SDK                                                    */
/* ================================================================== */
#else  /* !IDF_VER */

#include <string.h>
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include "tusb.h"
#include "flash_kv.h"

/* ---- One-time init (flash_kv_init is called from rp2350/main.c) ---- */
static inline void platform_init(void) { /* no-op */ }

/* ---- USB write ---- */
static inline void platform_usb_write(const uint8_t *buf, size_t len) {
    tud_cdc_write(buf, len);
    tud_cdc_write_flush();
}

/* ---- KV: get ---- */
static inline uint8_t  platform_kv_get_u8 (const char *key, uint8_t  def) {
    return flash_kv_get_u8(key, def);
}
static inline uint16_t platform_kv_get_u16(const char *key, uint16_t def) {
    return flash_kv_get_u16(key, def);
}
static inline bool platform_kv_get_str(const char *key, char *buf, size_t n) {
    return flash_kv_get_str(key, buf, n);
}

/* ---- KV: set ---- */
static inline void platform_kv_set_u8 (const char *key, uint8_t  val) {
    flash_kv_set_u8(key, val);
}
static inline void platform_kv_set_u16(const char *key, uint16_t val) {
    flash_kv_set_u16(key, val);
}
static inline void platform_kv_set_str(const char *key, const char *val) {
    flash_kv_set_str(key, val);
}
static inline void platform_kv_commit(void) {
    flash_kv_commit();
}

/* ---- Mutex (no-op — single core, main loop only) ---- */
typedef struct { int _dummy; } p8_mutex_t;

static inline void platform_mutex_init  (p8_mutex_t *m) { (void)m; }
static inline void platform_mutex_lock  (p8_mutex_t *m) { (void)m; }
static inline void platform_mutex_unlock(p8_mutex_t *m) { (void)m; }

/* ---- System ---- */
static inline void platform_sleep_ms(uint32_t ms) {
    sleep_ms(ms);
}
static inline void platform_restart(void) {
    watchdog_enable(1, 1);
    while (1) tight_loop_contents();
}

/* ---- Logging (disabled — stdio is off to keep P8 stream clean) ---- */
#define PLAT_LOGI(tag, fmt, ...)  ((void)0)
#define PLAT_LOGW(tag, fmt, ...)  ((void)0)
#define PLAT_LOGD(tag, fmt, ...)  ((void)0)

#endif  /* IDF_VER */
