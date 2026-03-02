/*
 * main.c — RP2350 Pulse-Eight CEC Dongle (XIAO RP2350)
 *
 * Wiring
 * ======
 *   HDMI CEC pin → GPIO29 (XIAO RP2350 pad D3)
 *   No external pull-up required — internal pull-up enabled in firmware.
 *   An optional external 27kΩ (in parallel → ~18kΩ) improves signal quality
 *   on long cable runs.
 *
 * USB
 * ===
 *   TinyUSB CDC with VID 0x2E8A (Raspberry Pi) / PID 0x1000 (placeholder).
 *   libCEC will auto-detect this device once the VID:PID is registered.
 *   Alternatively, specify the port explicitly:
 *
 *     cec-client -t p /dev/ttyACMx
 *     inputattach -p8 /dev/ttyACMx
 *
 * Execution model
 * ===============
 *   Single core (core 0), single main loop:
 *     tud_task()    — TinyUSB event pump
 *     CDC read      — feed bytes into P8 parser
 *     cec_rx_process() — drain edge ring buffer, decode CEC frames
 *     cec_bus_tick()   — start pending TX, deliver deferred TX results
 *
 *   GPIO IRQ: edge timestamps into ring buffer (never blocks main loop)
 *   Alarm 0: CEC TX phase transitions
 *   Alarm 1: ACK release
 *
 * XIAO RP2350 GPIO map (relevant pads)
 * =====================================
 *   D0=GPIO26  D1=GPIO27  D2=GPIO28  D3=GPIO29  D4=GPIO6
 *   D5=GPIO7   D6=GPIO0   D7=GPIO1   D8=GPIO2   D9=GPIO4  D10=GPIO3
 */

#include "pico/stdlib.h"
#include "tusb.h"

#include "cec_bus.h"
#include "p8_protocol.h"
#include "flash_kv.h"

/* CEC GPIO: XIAO RP2350 pad D3 = GPIO29. */
#define CEC_GPIO  29

/* ------------------------------------------------------------------ */
/* CEC receive callback → forward to P8 protocol                       */
/* ------------------------------------------------------------------ */

static void on_cec_frame(const cec_frame_t *frame) {
    p8_send_cec_frame(frame);
}

/* ------------------------------------------------------------------ */
/* main                                                                 */
/* ------------------------------------------------------------------ */

int main(void) {
    /* Load settings from flash (before p8_protocol_init reads them). */
    flash_kv_init();

    /* Initialise TinyUSB (descriptors in usb_descriptors.c). */
    tusb_init();

    /* Initialise CEC bus driver. */
    cec_bus_init(CEC_GPIO, on_cec_frame);

    /* Initialise P8 protocol handler (loads settings, sends fw version). */
    p8_protocol_init();

    while (true) {
        /* TinyUSB event pump — must be called regularly. */
        tud_task();

        /* Read CDC data from host and feed into P8 parser. */
        if (tud_cdc_connected()) {
            uint8_t buf[64];
            uint32_t n = tud_cdc_read(buf, sizeof(buf));
            for (uint32_t i = 0; i < n; i++) {
                p8_rx_byte(buf[i]);
            }
        }

        /* Decode CEC edges accumulated since last iteration. */
        cec_rx_process();

        /* Kick off any pending CEC TX; deliver deferred TX results. */
        cec_bus_tick();
    }

    return 0;
}
