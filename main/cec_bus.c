/*
 * cec_bus.c — CEC single-wire bit-bang driver for ESP32-C6.
 *
 * Architecture
 * ============
 *  RX:
 *    - GPIO configured open-drain with ANYEDGE interrupt.
 *    - The ISR timestamps every edge (us) and pushes an event to a queue.
 *    - A high-priority "CEC RX task" processes the events and decodes bits.
 *    - When a complete frame is ready it calls the user callback from that task.
 *    - ACK generation (driving bus LOW during the ACK slot) is done inside the
 *      ISR via esp_timer_start_once to keep latency minimal.
 *
 *  TX:
 *    - A "CEC TX task" waits on a queue for a frame to send.
 *    - It uses esp_timer_start_once callbacks to drive each waveform phase
 *      without busy-waiting, allowing the FreeRTOS scheduler to run between
 *      phases.
 *    - Arbitration is monitored: after releasing the bus for a bit '1', if the
 *      line is still LOW we lost arbitration and abort.
 *
 *  Concurrency:
 *    - s_tx_active prevents the RX decoder from acting on signals it
 *      generated itself.
 *    - The ISR only writes to edge-event queue; all decoding is in a task.
 */

#include "cec_bus.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "rom/ets_sys.h"   /* ets_delay_us for very short busy-waits in TX */

static const char *TAG = "cec_bus";

/* ------------------------------------------------------------------ */
/* Private state                                                        */
/* ------------------------------------------------------------------ */

static gpio_num_t  s_gpio;
static cec_rx_cb_t s_rx_cb;
static uint16_t    s_ack_mask = 0;
static volatile bool    s_tx_active = false;           /* suppress RX during our TX */
static volatile int64_t s_last_bus_activity_us = 0;    /* timestamp of last edge seen on the bus */

/* ---- RX ---------------------------------------------------------- */

typedef struct {
    int64_t time_us;   /* esp_timer_get_time() at the edge */
    int     level;     /* 0 = falling edge seen; 1 = rising edge seen */
} edge_event_t;

#define RX_EDGE_QUEUE_DEPTH 64
static QueueHandle_t   s_rx_edge_queue;
static TaskHandle_t    s_rx_task_handle;

/* ACK timer: drives the GPIO LOW for CEC_ACK_LOW_US, then releases. */
static esp_timer_handle_t s_ack_release_timer;
static esp_timer_handle_t s_ack_drive_timer;   /* drives LOW at start of ACK slot */

/* Flag set by RX decoder to trigger ACK generation for the next ACK slot. */
static volatile bool s_do_ack = false;

/* ---- TX ---------------------------------------------------------- */

typedef struct {
    uint8_t          data[CEC_MAX_FRAME_BYTES];
    uint8_t          len;
    uint8_t          idle_time;   /* signal-free periods to wait */
    bool             need_ack;
    cec_tx_done_cb_t done_cb;
} tx_request_t;

#define TX_QUEUE_DEPTH 4
static QueueHandle_t   s_tx_queue;
static TaskHandle_t    s_tx_task_handle;

/* TX timer used to chain waveform phases */
static esp_timer_handle_t s_tx_phase_timer;

/* TX state shared between the TX task and timer callbacks */
typedef enum {
    TX_PHASE_IDLE,
    TX_PHASE_START_LOW,
    TX_PHASE_START_HIGH,
    TX_PHASE_BIT_LOW,
    TX_PHASE_ARB_CHECK,   /* check arbitration at the CEC sample point */
    TX_PHASE_BIT_HIGH,
    TX_PHASE_EOM_LOW,
    TX_PHASE_EOM_HIGH,
    TX_PHASE_ACK_LOW,    /* driving bus LOW at the start of the ACK bit */
    TX_PHASE_ACK,
    TX_PHASE_DONE,
} tx_phase_t;

static volatile tx_phase_t s_tx_phase = TX_PHASE_IDLE;
static volatile bool s_tx_success;
static volatile uint8_t s_tx_fail_reason;

static tx_request_t s_cur_tx;
static int          s_tx_byte_idx;
static int          s_tx_bit_idx;   /* 7 = MSB first */
static int64_t      s_tx_bit_start; /* us timestamp of current bit's falling edge */
static SemaphoreHandle_t s_tx_done_sem;

/* ------------------------------------------------------------------ */
/* Helper: drive / release the CEC bus                                 */
/* ------------------------------------------------------------------ */

static inline void IRAM_ATTR bus_assert(void) {
    /* Drive LOW — dominant state */
    gpio_set_level(s_gpio, 0);
}

static inline void IRAM_ATTR bus_release(void) {
    /* Release to HIGH — recessive state (pull-up takes over) */
    gpio_set_level(s_gpio, 1);
}

static inline int IRAM_ATTR bus_read(void) {
    return gpio_get_level(s_gpio);
}

/* ------------------------------------------------------------------ */
/* ACK timer callbacks (ISR-safe context — use IRAM)                   */
/* ------------------------------------------------------------------ */

/*
 * Fires at the start of the ACK bit slot (falling edge from transmitter).
 * If we should ACK, drive the line LOW now.
 */
static void IRAM_ATTR ack_drive_cb(void *arg) {
    if (s_do_ack) {
        bus_assert();
        /* Release after CEC_ACK_LOW_US */
        esp_timer_start_once(s_ack_release_timer, CEC_ACK_LOW_US);
    }
    s_do_ack = false;
}

static void IRAM_ATTR ack_release_cb(void *arg) {
    bus_release();
}

/* ------------------------------------------------------------------ */
/* GPIO ISR                                                             */
/* ------------------------------------------------------------------ */

static void IRAM_ATTR cec_gpio_isr(void *arg) {
    if (s_tx_active) {
        /* Ignore edges caused by our own transmission. */
        return;
    }

    edge_event_t ev = {
        .time_us = esp_timer_get_time(),
        .level   = gpio_get_level(s_gpio),
    };

    /* Track last bus activity for idle-time detection in the TX task.
     * Every edge (rising or falling) counts — this gives accurate idle time
     * even for short 600µs '1'-bit LOWs that 1ms polling would miss. */
    s_last_bus_activity_us = ev.time_us;

    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(s_rx_edge_queue, &ev, &woken);
    if (woken) portYIELD_FROM_ISR();
}

/* ------------------------------------------------------------------ */
/* RX decoder task                                                      */
/* ------------------------------------------------------------------ */

/*
 * Bit sub-state within each CEC byte:
 *   bits 0-7 → data bits (MSB first)
 *   bit  8   → EOM
 *   bit  9   → ACK
 */
#define BIT_IDX_EOM  8
#define BIT_IDX_ACK  9

typedef enum {
    RXS_IDLE,        /* waiting for a start bit falling edge */
    RXS_START,       /* got falling edge; next edge must be the start-bit rise */
    RXS_BIT_LOW,     /* in the LOW portion of a data/EOM/ACK bit */
    RXS_BIT_HIGH,    /* waiting for next bit's falling edge */
} rxs_t;

static void cec_rx_task(void *arg) {
    rxs_t   state      = RXS_IDLE;
    int64_t fall_time  = 0;   /* us of last falling edge */

    uint8_t cur_byte   = 0;
    int     bit_idx    = 0;   /* 0-9 within current byte slot */
    bool    eom        = false;

    cec_frame_t frame;
    memset(&frame, 0, sizeof(frame));

    for (;;) {
        edge_event_t ev;
        /* Wait for an edge event (block forever) */
        xQueueReceive(s_rx_edge_queue, &ev, portMAX_DELAY);

        if (ev.level == 0) {
            /* ---- FALLING EDGE ---- */
            fall_time = ev.time_us;

            switch (state) {
            case RXS_IDLE:
                /* Could be start of a start bit */
                state = RXS_START;
                break;

            case RXS_BIT_HIGH:
                /*
                 * New bit starting.  The previous bit's value was already
                 * decoded on the rising edge.  Now check if we're in the ACK
                 * slot and should drive LOW.
                 */
                if (bit_idx == BIT_IDX_ACK) {
                    /*
                     * Decide whether to ACK this byte.
                     *
                     * The destination address lives in the LOW nibble of the
                     * HEADER byte (frame byte 0).  That byte is stored on the
                     * RISING edge of the ACK slot (in RXS_BIT_LOW below), but
                     * we need to decide on the FALLING edge (here).
                     *
                     * When frame.len == 0 (header byte not yet stored) the
                     * destination nibble is still live in cur_byte.
                     * When frame.len > 0 (subsequent bytes) use the already-
                     * stored header frame.data[0].
                     */
                    uint8_t dest = (frame.len == 0)
                                   ? (cur_byte & 0x0F)
                                   : (frame.data[0] & 0x0F);
                    if (s_ack_mask & (1u << dest)) {
                        s_do_ack = true;
                        /* Schedule ack_drive immediately (0us = ASAP) */
                        esp_timer_start_once(s_ack_drive_timer, 0);
                    }
                }
                state = RXS_BIT_LOW;
                break;

            case RXS_START:
                /* Unexpected fall while expecting start bit's rise — reset */
                state = RXS_IDLE;
                break;

            default:
                state = RXS_IDLE;
                break;
            }

        } else {
            /* ---- RISING EDGE ---- */
            int64_t low_dur = ev.time_us - fall_time;

            switch (state) {
            case RXS_START: {
                /* Validate start bit LOW duration */
                if (low_dur >= CEC_START_LOW_MIN_US && low_dur <= CEC_START_LOW_MAX_US) {
                    ESP_LOGD(TAG, "Start bit OK, low=%lld us", (long long)low_dur);
                    /* Begin receiving: reset frame state */
                    memset(&frame, 0, sizeof(frame));
                    cur_byte = 0;
                    bit_idx  = 0;
                    eom      = false;
                    state    = RXS_BIT_HIGH;
                } else {
                    ESP_LOGD(TAG, "Bad start bit low=%lld us, ignoring", (long long)low_dur);
                    state = RXS_IDLE;
                }
                break;
            }

            case RXS_BIT_LOW: {
                /* Decode bit value from low duration */
                int bit_val = (low_dur >= CEC_BIT_THRESHOLD_US) ? 0 : 1;

                if (bit_idx < 8) {
                    /* Data bit (MSB first) */
                    cur_byte = (cur_byte << 1) | bit_val;

                } else if (bit_idx == BIT_IDX_EOM) {
                    eom = (bit_val == 1);  /* EOM=1 means this is the last byte */

                } else if (bit_idx == BIT_IDX_ACK) {
                    /*
                     * ACK bit:
                     *   bit_val=0 → bus pulled LOW → frame was ACK'd
                     *   bit_val=1 → bus stayed HIGH → NAK / broadcast
                     */
                    frame.ack = (bit_val == 0);

                    /* Store the completed byte */
                    if (frame.len < CEC_MAX_FRAME_BYTES) {
                        frame.data[frame.len++] = cur_byte;
                    }

                    ESP_LOGD(TAG, "Byte[%d]=0x%02X eom=%d ack=%d",
                             frame.len - 1, cur_byte, eom, frame.ack);

                    cur_byte = 0;
                    bit_idx  = -1;  /* will be incremented to 0 below */

                    if (eom) {
                        /* Frame is complete — deliver to user */
                        if (s_rx_cb) {
                            s_rx_cb(&frame);
                        }
                        state = RXS_IDLE;
                        goto next_bit;  /* don't advance state to BIT_HIGH */
                    }
                }

                bit_idx++;
                state = RXS_BIT_HIGH;
                next_bit:;
                break;
            }

            default:
                /* Unexpected edge — stay in current state */
                break;
            }
        }
    }
}

/* ------------------------------------------------------------------ */
/* TX phase timer callback                                              */
/* ------------------------------------------------------------------ */

/*
 * Returns the number of microseconds until end-of-bit from the current
 * bit start timestamp.  Called from ISR context — must be IRAM_ATTR.
 */
static int64_t IRAM_ATTR time_to_bit_end(void) {
    int64_t elapsed = esp_timer_get_time() - s_tx_bit_start;
    int64_t remaining = CEC_BIT_PERIOD_US - elapsed;
    return (remaining > 0) ? remaining : 0;
}

/* Forward declaration */
static void tx_start_bit(void);
static void tx_next_bit(void);

static void IRAM_ATTR tx_phase_cb(void *arg) {
    switch (s_tx_phase) {

    case TX_PHASE_START_LOW:
        /* Start bit LOW phase done — release and wait for HIGH phase */
        bus_release();
        s_tx_phase = TX_PHASE_START_HIGH;
        esp_timer_start_once(s_tx_phase_timer, CEC_START_HIGH_US);
        break;

    case TX_PHASE_START_HIGH:
        /* Start bit done — begin first byte, MSB first */
        s_tx_byte_idx = 0;
        s_tx_bit_idx  = 7;
        tx_next_bit();
        break;

    case TX_PHASE_BIT_LOW: {
        /* Bit LOW phase done — release bus */
        bus_release();

        int current_bit = (s_cur_tx.data[s_tx_byte_idx] >> s_tx_bit_idx) & 1;
        if (current_bit == 1) {
            /*
             * Arbitration check for '1' bits: schedule it at the CEC sample
             * point (1050µs from the bit's falling edge) rather than right
             * here.  Checking immediately after bus_release() risks a false
             * positive because the pull-up needs ~5–25µs to charge the line
             * back to VCC.  At 1050µs the line is unambiguously HIGH unless
             * another device is actively holding it LOW.
             */
            int64_t elapsed    = esp_timer_get_time() - s_tx_bit_start;
            int64_t to_sample  = CEC_SAMPLE_US - elapsed;
            s_tx_phase = TX_PHASE_ARB_CHECK;
            esp_timer_start_once(s_tx_phase_timer, (to_sample > 10) ? to_sample : 10);
        } else {
            /* '0' bit: no arbitration possible, just wait out the bit period */
            s_tx_phase = TX_PHASE_BIT_HIGH;
            esp_timer_start_once(s_tx_phase_timer, time_to_bit_end());
        }
        break;
    }

    case TX_PHASE_ARB_CHECK: {
        /*
         * Sample point for arbitration on a '1' bit.
         * If the bus is still LOW, another device beat us — abort.
         */
        if (bus_read() == 0) {
            s_tx_active      = false;
            s_tx_success     = false;
            s_tx_fail_reason = 0x11; /* TRANSMIT_FAILED_LINE */
            s_tx_phase       = TX_PHASE_DONE;
            BaseType_t woken = pdFALSE;
            xSemaphoreGiveFromISR(s_tx_done_sem, &woken);
            if (woken) portYIELD_FROM_ISR();
            return;
        }
        /* Bus is HIGH — no arbitration loss, continue to HIGH phase */
        s_tx_phase = TX_PHASE_BIT_HIGH;
        esp_timer_start_once(s_tx_phase_timer, time_to_bit_end());
        break;
    }

    case TX_PHASE_BIT_HIGH:
        /* Bit period done — advance to next bit */
        s_tx_bit_idx--;
        if (s_tx_bit_idx < 0) {
            /* All 8 data bits sent — send EOM bit */
            s_tx_phase = TX_PHASE_EOM_LOW;
            s_tx_bit_start = esp_timer_get_time();
            bool is_last = (s_tx_byte_idx == s_cur_tx.len - 1);
            /* EOM=1 on last byte, EOM=0 otherwise */
            if (is_last) {
                bus_assert();
                esp_timer_start_once(s_tx_phase_timer, CEC_BIT1_LOW_US); /* '1' = short LOW */
            } else {
                bus_assert();
                esp_timer_start_once(s_tx_phase_timer, CEC_BIT0_LOW_US); /* '0' = long LOW  */
            }
        } else {
            tx_next_bit();
        }
        break;

    case TX_PHASE_EOM_LOW:
        /* EOM LOW done — release */
        bus_release();
        s_tx_phase = TX_PHASE_EOM_HIGH;
        esp_timer_start_once(s_tx_phase_timer, time_to_bit_end());
        break;

    case TX_PHASE_EOM_HIGH:
        /* EOM bit period done — begin ACK bit.
         * Per CEC spec the initiator drives a '1' bit (short LOW = 600µs) to
         * mark the start of the ACK slot.  The receiver uses this falling edge
         * as its reference: if it wants to ACK, it holds the bus LOW so that
         * at the sample point (1050µs from the falling edge) the bus is LOW. */
        s_tx_phase     = TX_PHASE_ACK_LOW;
        s_tx_bit_start = esp_timer_get_time();
        bus_assert();
        esp_timer_start_once(s_tx_phase_timer, CEC_BIT1_LOW_US); /* 600µs */
        break;

    case TX_PHASE_ACK_LOW:
        /* ACK bit LOW phase done — release and wait until the sample point.
         * 600µs has elapsed; sample point is at 1050µs → wait 450µs more. */
        bus_release();
        s_tx_phase = TX_PHASE_ACK;
        esp_timer_start_once(s_tx_phase_timer, CEC_SAMPLE_US - CEC_BIT1_LOW_US);
        break;

    case TX_PHASE_ACK: {
        /* Sample ACK */
        int ack_seen = (bus_read() == 0);  /* LOW = ACK'd */

        bool is_last = (s_tx_byte_idx == s_cur_tx.len - 1);

        if (s_cur_tx.need_ack && !ack_seen && is_last) {
            /* Direct message, last byte, not ACK'd → failure */
            /* Wait out the rest of the ACK bit period */
            esp_timer_start_once(s_tx_phase_timer, time_to_bit_end());
            s_tx_active     = false;
            s_tx_success    = false;
            s_tx_fail_reason = 0x12; /* TRANSMIT_FAILED_ACK */
            s_tx_phase      = TX_PHASE_DONE;
            /* Semaphore posted at end-of-period to let the bus settle */
        } else if (!is_last) {
            /* More bytes to send — wait for ACK bit to finish then continue.
             * Set bit_idx to 8 so TX_PHASE_BIT_HIGH decrements it to 7 (MSB)
             * before calling tx_next_bit().  Setting it to 7 here would cause
             * TX_PHASE_BIT_HIGH to decrement to 6 and skip the MSB. */
            s_tx_byte_idx++;
            s_tx_bit_idx = 8;
            esp_timer_start_once(s_tx_phase_timer, time_to_bit_end());
            s_tx_phase = TX_PHASE_BIT_HIGH;
        } else {
            /* Done! Wait out rest of ACK period */
            esp_timer_start_once(s_tx_phase_timer, time_to_bit_end());
            s_tx_active  = false;
            s_tx_success = true;
            s_tx_phase   = TX_PHASE_DONE;
        }
        break;
    }

    case TX_PHASE_DONE: {
        s_tx_active = false;
        bus_release();
        BaseType_t woken = pdFALSE;
        xSemaphoreGiveFromISR(s_tx_done_sem, &woken);
        if (woken) portYIELD_FROM_ISR();
        break;
    }

    default:
        break;
    }
}

/* Start transmitting one data/EOM bit.  Called from ISR context. */
static void IRAM_ATTR tx_next_bit(void) {
    int byte_val = s_cur_tx.data[s_tx_byte_idx];
    int bit_val  = (byte_val >> s_tx_bit_idx) & 1;

    s_tx_phase     = TX_PHASE_BIT_LOW;
    s_tx_bit_start = esp_timer_get_time();

    bus_assert();
    esp_timer_start_once(s_tx_phase_timer,
                         bit_val ? CEC_BIT1_LOW_US : CEC_BIT0_LOW_US);
}

/* ------------------------------------------------------------------ */
/* TX task                                                              */
/* ------------------------------------------------------------------ */

static void cec_tx_task(void *arg) {
    for (;;) {
        xQueueReceive(s_tx_queue, &s_cur_tx, portMAX_DELAY);

        /* ----------------------------------------------------------
         * Wait for the bus to be idle for idle_time signal-free periods.
         *
         * We use s_last_bus_activity_us (updated by the GPIO ISR on every
         * edge) rather than polling bus_read() at 1ms intervals.  Polling at
         * 1ms misses CEC '1'-bit LOW pulses (only 600µs) and can cause us to
         * start transmitting while another device is still mid-frame.
         *
         * A hard 3-second timeout prevents the task from blocking indefinitely
         * when the bus is permanently busy.  We report TRANSMIT_FAILED_TIMEOUT_LINE
         * (0x14) in that case.
         * ---------------------------------------------------------- */
        uint32_t required_idle_us = (uint32_t)s_cur_tx.idle_time * CEC_BIT_PERIOD_US;
        int64_t  wait_start_us    = esp_timer_get_time();
        bool     idle_ok          = false;

        while (true) {
            int64_t now = esp_timer_get_time();
            if ((now - s_last_bus_activity_us) >= required_idle_us) {
                idle_ok = true;
                break;
            }
            if ((now - wait_start_us) >= 3000000LL) { /* 3-second hard timeout */
                break;
            }
            vTaskDelay(1);
        }

        if (!idle_ok) {
            ESP_LOGW(TAG, "TX idle-wait timed out — bus busy");
            if (s_cur_tx.done_cb) {
                s_cur_tx.done_cb(false, 0x14); /* TRANSMIT_FAILED_TIMEOUT_LINE */
            }
            continue;
        }

        /* ----------------------------------------------------------
         * Begin transmission.
         * ---------------------------------------------------------- */
        s_tx_active  = true;
        s_tx_success = false;
        s_tx_fail_reason = 0;
        s_tx_phase   = TX_PHASE_START_LOW;
        s_tx_byte_idx = 0;
        s_tx_bit_idx  = 7;

        /* Drive start bit LOW */
        bus_assert();
        esp_timer_start_once(s_tx_phase_timer, CEC_START_LOW_US);

        /* Wait for transmission to finish */
        xSemaphoreTake(s_tx_done_sem, portMAX_DELAY);

        /* Invoke callback */
        if (s_cur_tx.done_cb) {
            s_cur_tx.done_cb(s_tx_success, s_tx_fail_reason);
        }
    }
}

/* ------------------------------------------------------------------ */
/* Public API                                                           */
/* ------------------------------------------------------------------ */

esp_err_t cec_bus_init(gpio_num_t gpio, cec_rx_cb_t rx_cb) {
    s_gpio  = gpio;
    s_rx_cb = rx_cb;

    /* Set last activity far enough in the past that the first TX can proceed
     * immediately without waiting for bus edges that will never come. */
    s_last_bus_activity_us = esp_timer_get_time()
                             - (int64_t)CEC_BIT_PERIOD_US * (CEC_DEFAULT_IDLE_PERIODS + 2);

    /* Configure GPIO as open-drain, input+output, interrupt on any edge */
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << gpio),
        .mode         = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en   = GPIO_PULLUP_ENABLE,    /* ~47kΩ internal pull-up; add external 27kΩ for long cables */
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_ANYEDGE,
    };
    esp_err_t ret = gpio_config(&cfg);
    if (ret != ESP_OK) return ret;

    /* Release the bus initially */
    bus_release();

    /* Create edge-event queue */
    s_rx_edge_queue = xQueueCreate(RX_EDGE_QUEUE_DEPTH, sizeof(edge_event_t));
    if (!s_rx_edge_queue) return ESP_ERR_NO_MEM;

    /* Create TX queue and semaphore */
    s_tx_queue = xQueueCreate(TX_QUEUE_DEPTH, sizeof(tx_request_t));
    if (!s_tx_queue) return ESP_ERR_NO_MEM;

    s_tx_done_sem = xSemaphoreCreateBinary();
    if (!s_tx_done_sem) return ESP_ERR_NO_MEM;

    /* Create ACK timers */
    esp_timer_create_args_t ack_drive_args = {
        .callback        = ack_drive_cb,
        .dispatch_method = ESP_TIMER_ISR,
        .name            = "cec_ack_drive",
    };
    ret = esp_timer_create(&ack_drive_args, &s_ack_drive_timer);
    if (ret != ESP_OK) return ret;

    esp_timer_create_args_t ack_release_args = {
        .callback        = ack_release_cb,
        .dispatch_method = ESP_TIMER_ISR,
        .name            = "cec_ack_rel",
    };
    ret = esp_timer_create(&ack_release_args, &s_ack_release_timer);
    if (ret != ESP_OK) return ret;

    /* Create TX phase timer */
    esp_timer_create_args_t tx_timer_args = {
        .callback        = tx_phase_cb,
        .dispatch_method = ESP_TIMER_ISR,
        .name            = "cec_tx",
    };
    ret = esp_timer_create(&tx_timer_args, &s_tx_phase_timer);
    if (ret != ESP_OK) return ret;

    /* Install GPIO ISR service and attach handler */
    gpio_install_isr_service(0);
    gpio_isr_handler_add(gpio, cec_gpio_isr, NULL);

    /* Start RX decoder task (priority 10 = high) */
    xTaskCreate(cec_rx_task, "cec_rx", 4096, NULL, 10, &s_rx_task_handle);

    /* Start TX task (priority 9) */
    xTaskCreate(cec_tx_task, "cec_tx", 4096, NULL, 9, &s_tx_task_handle);

    ESP_LOGI(TAG, "CEC bus initialised on GPIO%d", gpio);
    return ESP_OK;
}

esp_err_t cec_bus_transmit(const uint8_t *data, uint8_t len,
                            uint8_t idle_time, bool need_ack,
                            cec_tx_done_cb_t done_cb) {
    if (len == 0 || len > CEC_MAX_FRAME_BYTES) return ESP_ERR_INVALID_ARG;

    tx_request_t req = {
        .len       = len,
        .idle_time = idle_time ? idle_time : CEC_DEFAULT_IDLE_PERIODS,
        .need_ack  = need_ack,
        .done_cb   = done_cb,
    };
    memcpy(req.data, data, len);

    if (xQueueSend(s_tx_queue, &req, pdMS_TO_TICKS(100)) != pdTRUE) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void cec_bus_set_ack_mask(uint16_t mask) {
    s_ack_mask = mask;
}
