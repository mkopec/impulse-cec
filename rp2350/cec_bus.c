/*
 * cec_bus.c — CEC single-wire bit-bang driver for RP2350.
 *
 * Architecture
 * ============
 *  RX:
 *    - GPIO configured with pull-up; direction toggled for open-drain simulation.
 *    - GPIO IRQ captures every edge into a 64-entry ring buffer with timestamps.
 *    - cec_rx_process() drains the ring buffer from the main loop and runs the
 *      RX state machine.  ACK is driven directly from cec_rx_process() and
 *      released via hardware alarm 1.
 *
 *  TX:
 *    - cec_bus_transmit() stores the request; cec_bus_tick() (main loop) starts
 *      it when the bus has been idle long enough.
 *    - Hardware alarm 0 chains through waveform phases (mirrors ESP32-C6 timer ISR).
 *    - TX done is deferred: alarm sets s_tx_result_ready = true; cec_bus_tick()
 *      delivers the result so TinyUSB writes never happen from IRQ context.
 *
 *  GPIO open-drain simulation (RP2350 has no hardware OD mode):
 *    - bus_assert():  gpio direction = OUTPUT, output = 0 (drive LOW)
 *    - bus_release(): gpio direction = INPUT,  pull-up enabled (high-Z → HIGH)
 *
 *  Hardware alarms:
 *    - Alarm 0: TX phase transitions
 *    - Alarm 1: ACK release
 */

#include "cec_bus.h"

#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/sync.h"

/* ------------------------------------------------------------------ */
/* Alarm assignments                                                    */
/* ------------------------------------------------------------------ */

#define ALARM_TX       0
#define ALARM_ACK_REL  1

/* ------------------------------------------------------------------ */
/* Private state                                                        */
/* ------------------------------------------------------------------ */

static int          s_gpio;
static cec_rx_cb_t  s_rx_cb;
static uint16_t     s_ack_mask = 0;
static volatile bool     s_tx_active       = false;
static volatile uint64_t s_last_activity_us = 0;

/* ---- RX ring buffer (written by GPIO IRQ, read by main loop) ----- */

typedef struct {
    uint64_t time_us;
    int      level;
} edge_event_t;

#define RING_SIZE 64
static edge_event_t       s_ring[RING_SIZE];
static volatile uint32_t  s_ring_head = 0;   /* written by IRQ */
static uint32_t           s_ring_tail = 0;   /* read by main loop */

/* ACK: set by RX decoder, acted on in cec_rx_process for the ACK falling edge */
static bool s_do_ack = false;

/* ---- TX state ---------------------------------------------------- */

typedef struct {
    uint8_t          data[CEC_MAX_FRAME_BYTES];
    uint8_t          len;
    uint8_t          idle_time;
    bool             need_ack;
    cec_tx_done_cb_t done_cb;
} tx_request_t;

static tx_request_t       s_cur_tx;
static tx_request_t       s_pending_tx;
static volatile bool      s_tx_pending       = false;
static volatile bool      s_tx_result_ready  = false;
static volatile bool      s_tx_success;
static volatile uint8_t   s_tx_fail_reason;
static uint64_t           s_tx_wait_start_us = 0;

static int                s_tx_byte_idx;
static int                s_tx_bit_idx;
static uint64_t           s_tx_bit_start;

typedef enum {
    TX_PHASE_IDLE,
    TX_PHASE_START_LOW,
    TX_PHASE_START_HIGH,
    TX_PHASE_BIT_LOW,
    TX_PHASE_ARB_CHECK,
    TX_PHASE_BIT_HIGH,
    TX_PHASE_EOM_LOW,
    TX_PHASE_EOM_HIGH,
    TX_PHASE_ACK_LOW,
    TX_PHASE_ACK,
    TX_PHASE_DONE,
} tx_phase_t;

static volatile tx_phase_t s_tx_phase = TX_PHASE_IDLE;

/* ------------------------------------------------------------------ */
/* Open-drain bus helpers                                               */
/* ------------------------------------------------------------------ */

static inline void bus_assert(void) {
    /* Drive LOW: switch to output (output register pre-loaded with 0). */
    gpio_set_dir(s_gpio, GPIO_OUT);
}

static inline void bus_release(void) {
    /* Release: switch to input; pull-up holds the line HIGH. */
    gpio_set_dir(s_gpio, GPIO_IN);
}

static inline int bus_read(void) {
    return (int)gpio_get((uint)s_gpio);
}

/* ------------------------------------------------------------------ */
/* Alarm helpers                                                        */
/* ------------------------------------------------------------------ */

/* Schedule alarm relative to s_tx_bit_start (absolute, precise). */
static inline void alarm_tx_from_bit_start(uint32_t offset_us) {
    hardware_alarm_set_target(ALARM_TX,
        from_us_since_boot(s_tx_bit_start + offset_us));
}

/* Schedule alarm at current_time + delay_us. */
static inline void alarm_tx_after(uint32_t delay_us) {
    hardware_alarm_set_target(ALARM_TX, make_timeout_time_us(delay_us));
}

/* ------------------------------------------------------------------ */
/* TX: start next data/EOM bit (called from alarm IRQ)                 */
/* ------------------------------------------------------------------ */

static void tx_next_bit(void) {
    int bit_val = (s_cur_tx.data[s_tx_byte_idx] >> s_tx_bit_idx) & 1;

    s_tx_phase     = TX_PHASE_BIT_LOW;
    s_tx_bit_start = time_us_64();

    bus_assert();
    alarm_tx_from_bit_start(bit_val ? CEC_BIT1_LOW_US : CEC_BIT0_LOW_US);
}

/* ------------------------------------------------------------------ */
/* TX phase alarm callback (alarm 0)                                    */
/* ------------------------------------------------------------------ */

static void tx_phase_alarm_cb(uint alarm_num) {
    (void)alarm_num;

    switch (s_tx_phase) {

    case TX_PHASE_START_LOW:
        bus_release();
        s_tx_phase = TX_PHASE_START_HIGH;
        alarm_tx_after(CEC_START_HIGH_US);
        break;

    case TX_PHASE_START_HIGH:
        s_tx_byte_idx = 0;
        s_tx_bit_idx  = 7;
        tx_next_bit();
        break;

    case TX_PHASE_BIT_LOW: {
        bus_release();

        int current_bit = (s_cur_tx.data[s_tx_byte_idx] >> s_tx_bit_idx) & 1;
        if (current_bit == 1) {
            /* '1' bit: check arbitration at the CEC sample point. */
            s_tx_phase = TX_PHASE_ARB_CHECK;
            alarm_tx_from_bit_start(CEC_SAMPLE_US);
        } else {
            /* '0' bit: no arbitration, wait out the bit period. */
            s_tx_phase = TX_PHASE_BIT_HIGH;
            alarm_tx_from_bit_start(CEC_BIT_PERIOD_US);
        }
        break;
    }

    case TX_PHASE_ARB_CHECK:
        if (bus_read() == 0) {
            /* Bus still LOW — lost arbitration. */
            s_tx_active      = false;
            s_tx_success     = false;
            s_tx_fail_reason = 0x11; /* TRANSMIT_FAILED_LINE */
            s_tx_phase       = TX_PHASE_DONE;
            s_tx_result_ready = true;
        } else {
            s_tx_phase = TX_PHASE_BIT_HIGH;
            alarm_tx_from_bit_start(CEC_BIT_PERIOD_US);
        }
        break;

    case TX_PHASE_BIT_HIGH:
        s_tx_bit_idx--;
        if (s_tx_bit_idx < 0) {
            /* All 8 data bits sent — send EOM bit. */
            bool is_last = (s_tx_byte_idx == s_cur_tx.len - 1);
            s_tx_phase     = TX_PHASE_EOM_LOW;
            s_tx_bit_start = time_us_64();
            bus_assert();
            alarm_tx_from_bit_start(is_last ? CEC_BIT1_LOW_US : CEC_BIT0_LOW_US);
        } else {
            tx_next_bit();
        }
        break;

    case TX_PHASE_EOM_LOW:
        bus_release();
        s_tx_phase = TX_PHASE_EOM_HIGH;
        alarm_tx_from_bit_start(CEC_BIT_PERIOD_US);
        break;

    case TX_PHASE_EOM_HIGH:
        /* Begin ACK bit: transmitter drives '1' bit LOW (600µs). */
        s_tx_phase     = TX_PHASE_ACK_LOW;
        s_tx_bit_start = time_us_64();
        bus_assert();
        alarm_tx_from_bit_start(CEC_BIT1_LOW_US);
        break;

    case TX_PHASE_ACK_LOW:
        /* Release and wait for sample point (1050µs from falling edge). */
        bus_release();
        s_tx_phase = TX_PHASE_ACK;
        alarm_tx_from_bit_start(CEC_SAMPLE_US);
        break;

    case TX_PHASE_ACK: {
        int  ack_seen = (bus_read() == 0);
        bool is_last  = (s_tx_byte_idx == s_cur_tx.len - 1);

        if (s_cur_tx.need_ack && !ack_seen && is_last) {
            /* Direct message, last byte, not ACK'd → failure. */
            s_tx_active      = false;
            s_tx_success     = false;
            s_tx_fail_reason = 0x12; /* TRANSMIT_FAILED_ACK */
            s_tx_phase       = TX_PHASE_DONE;
            /* Wait for bit period to end before notifying main loop. */
            alarm_tx_from_bit_start(CEC_BIT_PERIOD_US);
        } else if (!is_last) {
            /* More bytes — continue after ACK bit period. */
            s_tx_byte_idx++;
            s_tx_bit_idx = 8;
            s_tx_phase   = TX_PHASE_BIT_HIGH;
            alarm_tx_from_bit_start(CEC_BIT_PERIOD_US);
        } else {
            /* Done! */
            s_tx_active  = false;
            s_tx_success = true;
            s_tx_phase   = TX_PHASE_DONE;
            alarm_tx_from_bit_start(CEC_BIT_PERIOD_US);
        }
        break;
    }

    case TX_PHASE_DONE:
        bus_release();
        s_tx_result_ready = true;
        break;

    default:
        break;
    }
}

/* ------------------------------------------------------------------ */
/* ACK release alarm callback (alarm 1)                                 */
/* ------------------------------------------------------------------ */

static void ack_release_alarm_cb(uint alarm_num) {
    (void)alarm_num;
    bus_release();
}

/* ------------------------------------------------------------------ */
/* GPIO IRQ — edge capture into ring buffer                             */
/* ------------------------------------------------------------------ */

static void cec_gpio_irq(uint gpio, uint32_t events) {
    if (s_tx_active) return;   /* ignore edges we generated ourselves */

    edge_event_t ev = {
        .time_us = time_us_64(),
        .level   = gpio_get(gpio) ? 1 : 0,
    };
    s_last_activity_us = ev.time_us;

    uint32_t next = (s_ring_head + 1) % RING_SIZE;
    if (next != s_ring_tail) {
        s_ring[s_ring_head] = ev;
        s_ring_head = next;
    }
}

/* ------------------------------------------------------------------ */
/* RX state machine constants                                           */
/* ------------------------------------------------------------------ */

#define BIT_IDX_EOM  8
#define BIT_IDX_ACK  9

typedef enum {
    RXS_IDLE,
    RXS_START,
    RXS_BIT_LOW,
    RXS_BIT_HIGH,
} rxs_t;

/* ------------------------------------------------------------------ */
/* Public: cec_rx_process — called from main loop                       */
/* ------------------------------------------------------------------ */

void cec_rx_process(void) {
    static rxs_t   state     = RXS_IDLE;
    static uint64_t fall_time = 0;
    static uint8_t  cur_byte  = 0;
    static int      bit_idx   = 0;
    static bool     eom       = false;
    static cec_frame_t frame;
    static bool     frame_init = false;

    if (!frame_init) {
        memset(&frame, 0, sizeof(frame));
        frame_init = true;
    }

    while (true) {
        /* Snapshot head (written by IRQ — volatile read). */
        uint32_t head = s_ring_head;
        if (s_ring_tail == head) break;   /* ring buffer empty */

        edge_event_t ev = s_ring[s_ring_tail];
        s_ring_tail = (s_ring_tail + 1) % RING_SIZE;

        if (ev.level == 0) {
            /* ---- FALLING EDGE ---- */
            fall_time = ev.time_us;

            switch (state) {
            case RXS_IDLE:
                state = RXS_START;
                break;

            case RXS_BIT_HIGH:
                if (bit_idx == BIT_IDX_ACK) {
                    /* ACK slot starting — decide whether to ACK. */
                    uint8_t dest = (frame.len == 0)
                                   ? (cur_byte & 0x0F)
                                   : (frame.data[0] & 0x0F);
                    if (s_ack_mask & (1u << dest)) {
                        s_do_ack = true;
                    }
                    if (s_do_ack) {
                        s_do_ack = false;
                        /* Drive LOW immediately from main loop.
                         * Main loop latency << 1050µs so we're still
                         * within the receiver drive window. */
                        bus_assert();
                        /* Release after CEC_ACK_LOW_US measured from
                         * the actual falling edge timestamp. */
                        hardware_alarm_set_target(ALARM_ACK_REL,
                            from_us_since_boot(ev.time_us + CEC_ACK_LOW_US));
                    }
                }
                state = RXS_BIT_LOW;
                break;

            case RXS_START:
                state = RXS_IDLE;
                break;

            default:
                state = RXS_IDLE;
                break;
            }

        } else {
            /* ---- RISING EDGE ---- */
            uint64_t low_dur = ev.time_us - fall_time;

            switch (state) {
            case RXS_START:
                if (low_dur >= CEC_START_LOW_MIN_US &&
                    low_dur <= CEC_START_LOW_MAX_US) {
                    memset(&frame, 0, sizeof(frame));
                    cur_byte = 0;
                    bit_idx  = 0;
                    eom      = false;
                    state    = RXS_BIT_HIGH;
                } else {
                    state = RXS_IDLE;
                }
                break;

            case RXS_BIT_LOW: {
                int bit_val = (low_dur >= CEC_BIT_THRESHOLD_US) ? 0 : 1;

                if (bit_idx < 8) {
                    cur_byte = (uint8_t)((cur_byte << 1) | bit_val);
                } else if (bit_idx == BIT_IDX_EOM) {
                    eom = (bit_val == 1);
                } else if (bit_idx == BIT_IDX_ACK) {
                    frame.ack = (bit_val == 0);

                    if (frame.len < CEC_MAX_FRAME_BYTES) {
                        frame.data[frame.len++] = cur_byte;
                    }

                    cur_byte = 0;
                    bit_idx  = -1;

                    if (eom) {
                        if (s_rx_cb) s_rx_cb(&frame);
                        state = RXS_IDLE;
                        goto next_bit;
                    }
                }

                bit_idx++;
                state = RXS_BIT_HIGH;
                next_bit:;
                break;
            }

            default:
                break;
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* Public: cec_bus_tick — called from main loop                         */
/* ------------------------------------------------------------------ */

void cec_bus_tick(void) {
    /* Deliver deferred TX result. */
    if (s_tx_result_ready) {
        s_tx_result_ready = false;
        if (s_cur_tx.done_cb) {
            s_cur_tx.done_cb(s_tx_success, s_tx_fail_reason);
        }
    }

    /* Start pending TX when bus is idle. */
    if (s_tx_pending && !s_tx_active) {
        uint64_t now          = time_us_64();
        uint32_t required_us  = (uint32_t)s_pending_tx.idle_time * CEC_BIT_PERIOD_US;

        if ((now - s_last_activity_us) >= required_us) {
            /* Bus idle long enough — begin transmission. */
            s_tx_pending     = false;
            s_cur_tx         = s_pending_tx;
            s_tx_active      = true;
            s_tx_success     = false;
            s_tx_fail_reason = 0;
            s_tx_phase       = TX_PHASE_START_LOW;
            s_tx_byte_idx    = 0;
            s_tx_bit_idx     = 7;

            bus_assert();
            alarm_tx_after(CEC_START_LOW_US);

        } else if ((now - s_tx_wait_start_us) >= 500000ULL) {
            /* Hard 500ms timeout — bus permanently busy. */
            s_tx_pending = false;
            if (s_pending_tx.done_cb) {
                s_pending_tx.done_cb(false, 0x14); /* TRANSMIT_FAILED_TIMEOUT_LINE */
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

esp_err_t cec_bus_init(gpio_num_t gpio, cec_rx_cb_t rx_cb) {
    s_gpio  = gpio;
    s_rx_cb = rx_cb;

    /* Initialise last activity far enough in the past that the first TX
     * can proceed immediately without waiting for bus edges. */
    s_last_activity_us = time_us_64()
                         - (uint64_t)CEC_BIT_PERIOD_US * (CEC_DEFAULT_IDLE_PERIODS + 2);

    /* Configure GPIO: pull-up enabled, output register pre-loaded LOW.
     * Direction starts as INPUT (released / recessive). */
    gpio_init((uint)gpio);
    gpio_pull_up((uint)gpio);
    gpio_put((uint)gpio, 0);          /* pre-load output reg: will be LOW when output */
    gpio_set_dir((uint)gpio, GPIO_IN); /* start released */

    /* Claim and configure hardware alarms. */
    hardware_alarm_claim(ALARM_TX);
    hardware_alarm_set_callback(ALARM_TX, tx_phase_alarm_cb);

    hardware_alarm_claim(ALARM_ACK_REL);
    hardware_alarm_set_callback(ALARM_ACK_REL, ack_release_alarm_cb);

    /* Register GPIO IRQ for both edges. */
    gpio_set_irq_enabled_with_callback((uint)gpio,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,
        cec_gpio_irq);

    return ESP_OK;
}

esp_err_t cec_bus_transmit(const uint8_t *data, uint8_t len,
                            uint8_t idle_time, bool need_ack,
                            cec_tx_done_cb_t done_cb) {
    if (len == 0 || len > CEC_MAX_FRAME_BYTES) return ESP_ERR_INVALID_ARG;
    if (s_tx_pending || s_tx_active)            return ESP_ERR_NO_MEM;

    s_pending_tx.len       = len;
    s_pending_tx.idle_time = idle_time ? idle_time : CEC_DEFAULT_IDLE_PERIODS;
    s_pending_tx.need_ack  = need_ack;
    s_pending_tx.done_cb   = done_cb;
    memcpy(s_pending_tx.data, data, len);

    s_tx_wait_start_us = time_us_64();
    s_tx_pending       = true;
    return ESP_OK;
}

void cec_bus_set_ack_mask(uint16_t mask) {
    s_ack_mask = mask;
}
