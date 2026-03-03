/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/*
 * main.c — ESP32-C6 Pulse-Eight CEC Dongle
 *
 * Wiring
 * ======
 *   HDMI CEC pin → GPIO21 (XIAO D3)
 *   No external pull-up required — the driver enables the ESP32-C6 internal
 *   ~47kΩ pull-up.  An optional external 27kΩ (in parallel → ~18kΩ) improves
 *   signal integrity on long cable runs.
 *
 * USB
 * ===
 *   The ESP32-C6 has a USB Serial/JTAG peripheral (not full USB OTG), so it
 *   presents as a fixed CDC ACM device with Espressif's VID:PID (0x303A:0x1001).
 *   libCEC won't auto-detect it as Pulse-Eight, so tell it the port explicitly:
 *
 *     cec-client -t p /dev/ttyACM0
 *     # macOS: /dev/cu.usbmodem*
 *     # Windows: COMx
 *
 *   The P8 binary protocol works identically over any CDC serial port;
 *   the VID:PID only matters for auto-detection.
 *
 * XIAO ESP32-C6 GPIO map
 * ======================
 *   D0=GPIO0  D1=GPIO1  D2=GPIO2  D3=GPIO21 D4=GPIO22
 *   D5=GPIO23 D6=GPIO16 D7=GPIO17 D8=GPIO19 D9=GPIO20 D10=GPIO18
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "driver/usb_serial_jtag.h"

#include "cec_bus.h"
#include "p8_protocol.h"

static const char *TAG = "main";

#define CEC_GPIO  CEC_DEFAULT_GPIO   /* GPIO5 = XIAO D3 */

/* ------------------------------------------------------------------ */
/* CEC receive callback → forward to P8 protocol                       */
/* ------------------------------------------------------------------ */

static void on_cec_frame(const cec_frame_t *frame) {
    ESP_LOGI(TAG, "CEC RX %d bytes, hdr=0x%02X", frame->len, frame->data[0]);
    p8_send_cec_frame(frame);
}

/* ------------------------------------------------------------------ */
/* USB RX task — reads bytes from USB Serial/JTAG and feeds the parser */
/* ------------------------------------------------------------------ */

static void usb_rx_task(void *arg) {
    uint8_t buf[64];
    for (;;) {
        int n = usb_serial_jtag_read_bytes(buf, sizeof(buf), pdMS_TO_TICKS(100));
        for (int i = 0; i < n; i++) {
            p8_rx_byte(buf[i]);
        }
    }
}

/* ------------------------------------------------------------------ */
/* app_main                                                             */
/* ------------------------------------------------------------------ */

void app_main(void) {
    ESP_LOGI(TAG, "ESP32-C6 Pulse-Eight CEC dongle starting");

    /* --- NVS --- */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS erase + reinit");
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    /* --- USB Serial/JTAG driver --- */
    usb_serial_jtag_driver_config_t usj_cfg = {
        .tx_buffer_size = 512,
        .rx_buffer_size = 512,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usj_cfg));
    ESP_LOGI(TAG, "USB Serial/JTAG ready");

    /* --- CEC bus driver --- */
    ESP_ERROR_CHECK(cec_bus_init(CEC_GPIO, on_cec_frame));

    /* --- P8 protocol handler --- */
    ESP_ERROR_CHECK(p8_protocol_init());

    /* --- USB RX task --- */
    /*
     * Priority 12 — above cec_tx_task (11) and cec_rx_task (10).
     *
     * usb_rx_task is the only path that processes P8 commands and sends
     * COMMAND_ACCEPTED.  The kernel pulse8 driver waits 1 second for each
     * response; if usb_rx_task is starved of s_tx_mutex the response times out.
     * At priority 12, FreeRTOS priority inheritance ensures the mutex holder
     * is boosted so the response is sent well within that 1-second window.
     */
    xTaskCreate(usb_rx_task, "usb_rx", 4096, NULL, 12, NULL);

    ESP_LOGI(TAG, "Ready. CEC on GPIO%d. "
             "Connect with: cec-client -t p /dev/ttyACM0", CEC_GPIO);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
