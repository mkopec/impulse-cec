#pragma once

/*
 * CEC bus driver — bit-bangs the HDMI CEC single-wire protocol.
 *
 * Hardware: connect the HDMI CEC pin directly to CEC_DEFAULT_GPIO.
 * The driver enables the ESP32-C6 internal ~47kΩ pull-up, so no external
 * resistor is required for short cables / direct connections.  For long
 * cable runs or noisy environments, add an external 27kΩ pull-up from
 * the CEC line to 3.3V (the two resistors simply parallel to ~18kΩ).
 *
 * The GPIO is configured as open-drain:
 *   - output LOW  → drives bus dominant (logic 0)
 *   - output HIGH → releases bus (pull-up holds it recessive / logic 1)
 *
 * CEC bit timing (all times in µs):
 *   Start bit:  LOW 3700µs, HIGH 800µs  (total 4500µs)
 *   Bit '0':    LOW 1500µs, HIGH 900µs  (total 2400µs)
 *   Bit '1':    LOW  600µs, HIGH 1800µs (total 2400µs)
 *   Sample at 1050µs from the falling edge (LOW → HIGH after this point = '1')
 *   ACK bit:    like bit '0' when ACK-ing; receiver pulls LOW 1500µs
 */

#include <stdint.h>
#include <stdbool.h>

#ifdef IDF_VER
#  include "esp_err.h"
#  include "driver/gpio.h"
#else
   typedef int  gpio_num_t;
   typedef int  esp_err_t;
#  define ESP_OK              0
#  define ESP_ERR_NO_MEM     -1
#  define ESP_ERR_INVALID_ARG -2
#endif

/* GPIO to use for CEC.  XIAO ESP32-C6 pad D3 = GPIO21; XIAO RP2350 pad D3 = GPIO5. */
#define CEC_DEFAULT_GPIO    21

/* Maximum number of payload bytes in one CEC frame (header + 14 data + EOM). */
#define CEC_MAX_FRAME_BYTES 16

/* ------------------------------------------------------------------ */
/* CEC timing constants (microseconds)                                 */
/* ------------------------------------------------------------------ */
#define CEC_START_LOW_US        3700
#define CEC_START_HIGH_US        800
#define CEC_START_TOTAL_US      4500
#define CEC_BIT_PERIOD_US       2400
#define CEC_BIT0_LOW_US         1500
#define CEC_BIT1_LOW_US          600
#define CEC_SAMPLE_US           1050    /* sample point after falling edge */
#define CEC_ACK_LOW_US          1500

/* Detection thresholds */
#define CEC_START_LOW_MIN_US    3300
#define CEC_START_LOW_MAX_US    4100
#define CEC_BIT_THRESHOLD_US    1050   /* LOW duration > this → bit '0' */

/* Minimum signal-free periods before transmitting (7 for new initiator) */
#define CEC_DEFAULT_IDLE_PERIODS  7

/* ------------------------------------------------------------------ */
/* Public types                                                         */
/* ------------------------------------------------------------------ */

/** A received CEC frame (all bytes, header first). */
typedef struct {
    uint8_t data[CEC_MAX_FRAME_BYTES];
    uint8_t len;        /* number of bytes (≥ 1, header counts) */
    bool    ack;        /* true if the frame was ACK'd on the bus */
} cec_frame_t;

/**
 * Callback invoked (from a task or main loop) when a complete CEC frame
 * has been received.
 */
typedef void (*cec_rx_cb_t)(const cec_frame_t *frame);

/**
 * Callback invoked when a previously requested transmission has
 * finished (success or failure).
 *
 * @param success  true  → TRANSMIT_SUCCEEDED
 *                 false → TRANSMIT_FAILED_*
 * @param reason   non-zero failure code (P8 MSGCODE_TRANSMIT_FAILED_*)
 *                 when success is false.
 */
typedef void (*cec_tx_done_cb_t)(bool success, uint8_t reason);

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

/**
 * Initialise the CEC bus driver.
 *
 * @param gpio    GPIO number connected to the CEC line.
 * @param rx_cb  Called from a task when a full frame is received.
 */
esp_err_t cec_bus_init(gpio_num_t gpio, cec_rx_cb_t rx_cb);

/**
 * Queue a CEC frame for transmission.
 *
 * @param data        Bytes to send, header byte first.
 * @param len         Number of bytes (1–16).
 * @param idle_time   Signal-free periods to wait before asserting the bus.
 * @param need_ack    true for direct messages (expect ACK from destination).
 * @param done_cb     Called (from a task) when the transmission finishes.
 *
 * @return ESP_OK if queued, ESP_ERR_NO_MEM / ESP_ERR_INVALID_STATE otherwise.
 */
esp_err_t cec_bus_transmit(const uint8_t *data, uint8_t len,
                            uint8_t idle_time, bool need_ack,
                            cec_tx_done_cb_t done_cb);

/**
 * Set the bitmask of logical addresses this adapter owns.
 * Frames addressed to these addresses will be ACK'd on the bus.
 * Default: 0 (do not ACK anything).
 *
 * Bit N corresponds to logical address N (bits 0–15).
 */
void cec_bus_set_ack_mask(uint16_t mask);

#ifndef IDF_VER
/**
 * RP2350 only: drain the GPIO edge ring buffer and run the RX decoder.
 * Call from the main loop on every iteration.
 */
void cec_rx_process(void);

/**
 * RP2350 only: start a pending TX when the bus is idle; deliver deferred
 * TX results.  Call from the main loop on every iteration.
 */
void cec_bus_tick(void);
#endif
