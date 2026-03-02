/*
 * p8_protocol.c — Pulse-Eight USB CEC adapter serial protocol handler.
 *
 * Wire format (libCEC 7.x, symmetric — same framing both directions):
 *
 *   MSGSTART (0xFF)  cmd_byte  [escaped_params...]  MSGEND (0xFE)
 *
 * Bytes >= 0xFD are escaped: MSGESC (0xFD) followed by (byte - 3).
 *
 * Transmit flow:
 *   1. Host sends TRANSMIT_IDLETIME  → sets idle_time
 *   2. Host sends TRANSMIT_ACK_POLARITY → sets need_ack
 *   3. Host sends TRANSMIT (n-1 times) + TRANSMIT_EOM (last byte)
 *   4. After TRANSMIT_EOM we call cec_bus_transmit()
 *   5. CEC bus callback → TRANSMIT_SUCCEEDED or TRANSMIT_FAILED_*
 */

#include "p8_protocol.h"
#include "platform.h"

#include <string.h>

static const char *TAG = "p8";

/* ------------------------------------------------------------------ */
/* Adapter settings (persisted in KV store)                             */
/* ------------------------------------------------------------------ */

typedef struct {
    uint8_t  auto_enabled;
    uint8_t  auto_power_on;
    uint8_t  default_logical_addr;
    uint16_t logical_addr_mask;
    uint16_t physical_addr;
    uint8_t  device_type;
    uint8_t  hdmi_version;
    char     osd_name[14];
} p8_settings_t;

static p8_settings_t s_cfg;

static void settings_load(void) {
    s_cfg.auto_enabled         = platform_kv_get_u8("auto_en",    1);
    s_cfg.auto_power_on        = platform_kv_get_u8("auto_pwr",   0);
    s_cfg.default_logical_addr = platform_kv_get_u8("def_la",     0x0F);
    s_cfg.logical_addr_mask    = platform_kv_get_u16("la_mask",   0x0000);
    s_cfg.physical_addr        = platform_kv_get_u16("phys_addr", 0x1000);
    s_cfg.device_type          = platform_kv_get_u8("dev_type",   4);
    s_cfg.hdmi_version         = platform_kv_get_u8("hdmi_ver",   0x05);

    if (!platform_kv_get_str("osd_name", s_cfg.osd_name, sizeof(s_cfg.osd_name))) {
        strlcpy(s_cfg.osd_name, "ESP32-P8", sizeof(s_cfg.osd_name));
    }
}

static void settings_save(void) {
    platform_kv_set_u8 ("auto_en",   s_cfg.auto_enabled);
    platform_kv_set_u8 ("auto_pwr",  s_cfg.auto_power_on);
    platform_kv_set_u8 ("def_la",    s_cfg.default_logical_addr);
    platform_kv_set_u16("la_mask",   s_cfg.logical_addr_mask);
    platform_kv_set_u16("phys_addr", s_cfg.physical_addr);
    platform_kv_set_u8 ("dev_type",  s_cfg.device_type);
    platform_kv_set_u8 ("hdmi_ver",  s_cfg.hdmi_version);
    platform_kv_set_str("osd_name",  s_cfg.osd_name);
    platform_kv_commit();
}

/* ------------------------------------------------------------------ */
/* USB write helpers                                                    */
/* ------------------------------------------------------------------ */

static p8_mutex_t s_tx_mutex;

/*
 * Write buffer — large enough for a 16-byte CEC frame where every byte
 * needs escaping: 16 * (MSGSTART + cmd + ESC + data + MSGEND) = 16*5 = 80.
 */
#define WRITE_BUF_SIZE  128
static uint8_t s_wbuf[WRITE_BUF_SIZE];
static int     s_wbuf_len = 0;

static void wb_put(uint8_t b) {
    if (s_wbuf_len < WRITE_BUF_SIZE) {
        s_wbuf[s_wbuf_len++] = b;
    }
}

static void wb_flush(void) {
    if (s_wbuf_len > 0) {
        platform_usb_write(s_wbuf, s_wbuf_len);
    }
    s_wbuf_len = 0;
    platform_mutex_unlock(&s_tx_mutex);
}

/* Lock mutex and begin a message: emit MSGSTART then cmd_byte. */
static void msg_begin(uint8_t cmd_byte) {
    platform_mutex_lock(&s_tx_mutex);
    s_wbuf_len = 0;
    wb_put(P8_MSGSTART);
    wb_put(cmd_byte);
}

/* Append a parameter byte with P8 escaping (bytes >= 0xFD → [MSGESC, byte-3]). */
static void msg_put_escaped(uint8_t b) {
    if (b >= P8_MSGESC) {
        wb_put(P8_MSGESC);
        wb_put(b - P8_ESCOFFSET);
    } else {
        wb_put(b);
    }
}

/* Emit MSGEND; must be followed by wb_flush() to transmit and release mutex. */
static void msg_end(void) {
    wb_put(P8_MSGEND);
}

/* ------------------------------------------------------------------ */
/* Common responses                                                     */
/* ------------------------------------------------------------------ */

static void send_accepted(void) {
    msg_begin(MSGCODE_COMMAND_ACCEPTED);
    msg_end();
    wb_flush();
}

static void send_rejected(void) {
    msg_begin(MSGCODE_COMMAND_REJECTED);
    msg_end();
    wb_flush();
}

/*
 * COMMAND_ACCEPTED with the parameter byte echoed back.
 * The kernel pulse8-cec driver checks pulse8->len >= size+1 for any
 * parameterised SET command; a bare ACCEPTED (len=1) fails that check.
 * The real P8 firmware echoes the parameter, giving len=2.
 */
static void send_accepted_echo(uint8_t v) {
    msg_begin(MSGCODE_COMMAND_ACCEPTED);
    msg_put_escaped(v);
    msg_end();
    wb_flush();
}

/* ------------------------------------------------------------------ */
/* Received CEC frame → send to host                                    */
/* ------------------------------------------------------------------ */

void p8_send_cec_frame(const cec_frame_t *frame) {
    if (frame->len == 0) return;

    platform_mutex_lock(&s_tx_mutex);
    s_wbuf_len = 0;

    /*
     * CEC ACK semantics differ for unicast vs broadcast:
     *   Unicast:   receiver drives bus LOW  → frame->ack=true  → P8_ACK_FLAG set
     *   Broadcast: nobody drives LOW (= all received OK) → frame->ack=false → P8_ACK_FLAG must
     *              still be SET, because the kernel driver maps a missing P8_ACK_FLAG to
     *              CEC_RX_STATUS_NACK and discards the frame.
     *              (A LOW during a broadcast ACK slot signals an error, i.e. frame->ack=true = bad.)
     */
    bool is_broadcast = (frame->data[0] & 0x0F) == 0x0F;
    bool ack_ok = is_broadcast ? !frame->ack : frame->ack;

    for (int i = 0; i < frame->len; i++) {
        bool is_last = (i == frame->len - 1);

        uint8_t cmd = (i == 0) ? MSGCODE_FRAME_START : MSGCODE_FRAME_DATA;
        if (is_last) cmd |= P8_EOM_FLAG;
        if (ack_ok)  cmd |= P8_ACK_FLAG;

        wb_put(P8_MSGSTART);
        wb_put(cmd);
        msg_put_escaped(frame->data[i]);
        wb_put(P8_MSGEND);
    }

    platform_usb_write(s_wbuf, s_wbuf_len);
    s_wbuf_len = 0;
    platform_mutex_unlock(&s_tx_mutex);
}

/* ------------------------------------------------------------------ */
/* CEC TX done callback                                                 */
/* ------------------------------------------------------------------ */

static void cec_tx_done(bool success, uint8_t reason) {
    if (success) {
        msg_begin(MSGCODE_TRANSMIT_SUCCEEDED);
    } else {
        msg_begin(reason ? reason : MSGCODE_TRANSMIT_FAILED_LINE);
    }
    msg_end();
    wb_flush();
}

/* ------------------------------------------------------------------ */
/* Command parser state                                                 */
/* ------------------------------------------------------------------ */

/*
 * Wire format from host:
 *   [0xFF MSGSTART] [cmd_byte] [param_bytes...] [0xFE MSGEND]
 * Bytes >= 0xFD are escaped: [0xFD MSGESC] [byte - 3].
 */

#define MSG_BUF_SIZE  64

static uint8_t s_msg_buf[MSG_BUF_SIZE];
static int     s_msg_len  = 0;
static bool    s_in_esc   = false;  /* previous byte was MSGESC */

/* Pending transmit accumulator */
typedef struct {
    uint8_t buf[CEC_MAX_FRAME_BYTES];
    uint8_t len;
    uint8_t idle_time;
    bool    need_ack;
} tx_accum_t;

static tx_accum_t s_tx = { .idle_time = 7, .need_ack = true };

/* ------------------------------------------------------------------ */
/* Command dispatch                                                     */
/* ------------------------------------------------------------------ */

static void handle_message(const uint8_t *msg, int len) {
    if (len == 0) return;

    /* bits 5:0 = command code; bits 7:6 = EOM/ACK flags (adapter→host only) */
    uint8_t code      = msg[0] & ~(P8_EOM_FLAG | P8_ACK_FLAG);
    bool    has_param = (len >= 2);
    uint8_t param     = has_param ? msg[1] : 0;

    PLAT_LOGD(TAG, "CMD 0x%02X param=0x%02X len=%d", code, param, len);

    switch (code) {

    case MSGCODE_PING:
        /*
         * Respond with COMMAND_ACCEPTED only — do NOT send FIRMWARE_VERSION.
         * The kernel pulse8_send_and_wait wakes on the first complete message;
         * an unsolicited FIRMWARE_VERSION would desynchronise all subsequent
         * responses.
         */
        send_accepted();
        break;

    case MSGCODE_FIRMWARE_VERSION:
        msg_begin(MSGCODE_FIRMWARE_VERSION);
        msg_put_escaped(P8_FIRMWARE_VERSION_MAJOR);
        msg_put_escaped(P8_FIRMWARE_VERSION_MINOR);
        msg_end();
        wb_flush();
        break;

    case MSGCODE_GET_BUILDDATE:
        msg_begin(MSGCODE_GET_BUILDDATE);
        msg_put_escaped(0x00);
        msg_put_escaped(0x00);
        msg_put_escaped(0x00);
        msg_put_escaped(0x00);
        msg_end();
        wb_flush();
        break;

    case MSGCODE_GET_ADAPTER_TYPE:
        msg_begin(MSGCODE_GET_ADAPTER_TYPE);
        msg_put_escaped(ADAPTERTYPE_USBCEC);
        msg_end();
        wb_flush();
        break;

    case MSGCODE_SET_ACK_MASK:
        if (len >= 3) {
            uint16_t mask = ((uint16_t)msg[1] << 8) | msg[2];
            s_cfg.logical_addr_mask = mask;
            cec_bus_set_ack_mask(mask);
            send_accepted_echo(msg[1]);
        } else {
            send_rejected();
        }
        break;

    /* ---- Transmit sequence ---- */
    case MSGCODE_TRANSMIT_IDLETIME:
        s_tx.idle_time = has_param ? param : 7;
        send_accepted_echo(param);
        break;

    case MSGCODE_TRANSMIT_ACK_POLARITY:
        /* 0 = unicast (receiver ACKs by pulling LOW); 1 = broadcast (no ACK) */
        s_tx.need_ack = has_param ? (param == 0) : true;
        send_accepted_echo(param);
        break;

    case MSGCODE_TRANSMIT_LINE_TIMEOUT:
        send_accepted_echo(param);
        break;

    case MSGCODE_TRANSMIT:
        if (!has_param) { send_rejected(); break; }
        if (s_tx.len < CEC_MAX_FRAME_BYTES) {
            s_tx.buf[s_tx.len++] = param;
        }
        send_accepted_echo(param);
        break;

    case MSGCODE_TRANSMIT_EOM:
        if (!has_param) { send_rejected(); break; }
        if (s_tx.len < CEC_MAX_FRAME_BYTES) {
            s_tx.buf[s_tx.len++] = param;
        }
        send_accepted_echo(param);

        PLAT_LOGI(TAG, "TX %d bytes idle=%d ack=%d",
                 s_tx.len, s_tx.idle_time, s_tx.need_ack);
        {
            esp_err_t err = cec_bus_transmit(s_tx.buf, s_tx.len,
                                              s_tx.idle_time, s_tx.need_ack,
                                              cec_tx_done);
            if (err != ESP_OK) {
                msg_begin(MSGCODE_TRANSMIT_FAILED_LINE);
                msg_end();
                wb_flush();
            }
        }
        s_tx.len = 0;
        break;

    /* ---- Settings ---- */
    case MSGCODE_SET_CONTROLLED:
        send_accepted_echo(param);
        break;

    case MSGCODE_GET_AUTO_ENABLED:
        /*
         * Always return 1 (autonomous mode), regardless of KV store.
         * libCEC writes 0 via SET_AUTO_ENABLED (controlled mode), but the
         * kernel pulse8-cec driver only calls cec_s_phys_addr — which triggers
         * logical address claiming — when pulse8->autonomous is true.
         */
        msg_begin(MSGCODE_GET_AUTO_ENABLED);
        msg_put_escaped(1);
        msg_end();
        wb_flush();
        break;

    case MSGCODE_SET_AUTO_ENABLED:
        s_cfg.auto_enabled = param;
        send_accepted_echo(param);
        break;

    case MSGCODE_GET_AUTO_POWER_ON:
        msg_begin(MSGCODE_GET_AUTO_POWER_ON);
        msg_put_escaped(s_cfg.auto_power_on);
        msg_end();
        wb_flush();
        break;

    case MSGCODE_SET_AUTO_POWER_ON:
        s_cfg.auto_power_on = param;
        send_accepted_echo(param);
        break;

    case MSGCODE_GET_DEFAULT_LOGICAL_ADDRESS:
        msg_begin(MSGCODE_GET_DEFAULT_LOGICAL_ADDRESS);
        msg_put_escaped(s_cfg.default_logical_addr);
        msg_end();
        wb_flush();
        break;

    case MSGCODE_SET_DEFAULT_LOGICAL_ADDRESS:
        s_cfg.default_logical_addr = param & 0x0F;
        send_accepted_echo(param);
        break;

    case MSGCODE_GET_LOGICAL_ADDRESS_MASK:
        msg_begin(MSGCODE_GET_LOGICAL_ADDRESS_MASK);
        msg_put_escaped((s_cfg.logical_addr_mask >> 8) & 0xFF);
        msg_put_escaped(s_cfg.logical_addr_mask & 0xFF);
        msg_end();
        wb_flush();
        break;

    case MSGCODE_SET_LOGICAL_ADDRESS_MASK:
        if (len >= 3) {
            s_cfg.logical_addr_mask = ((uint16_t)msg[1] << 8) | msg[2];
            /* Store only — do NOT call cec_bus_set_ack_mask().  The live ACK
             * mask is set exclusively by SET_ACK_MASK, after libCEC self-POLLs
             * confirm a free address. */
            send_accepted_echo(msg[1]);
        } else {
            send_rejected();
        }
        break;

    case MSGCODE_GET_PHYSICAL_ADDRESS: {
        /*
         * Never return 0xffff (CEC_PHYS_ADDR_INVALID).  The kernel's
         * adap_s_phys_addr callback writes 0xffff during adapter registration,
         * before pulse8_setup reads our PA.  Fall back to 1.0.0.0 (0x1000).
         */
        uint16_t pa = (s_cfg.physical_addr == 0xffff) ? 0x1000
                                                       : s_cfg.physical_addr;
        msg_begin(MSGCODE_GET_PHYSICAL_ADDRESS);
        msg_put_escaped((pa >> 8) & 0xFF);
        msg_put_escaped(pa & 0xFF);
        msg_end();
        wb_flush();
        break;
    }

    case MSGCODE_SET_PHYSICAL_ADDRESS:
        if (len >= 3) {
            s_cfg.physical_addr = ((uint16_t)msg[1] << 8) | msg[2];
            send_accepted_echo(msg[1]);
        } else {
            send_rejected();
        }
        break;

    case MSGCODE_GET_DEVICE_TYPE:
        msg_begin(MSGCODE_GET_DEVICE_TYPE);
        msg_put_escaped(s_cfg.device_type);
        msg_end();
        wb_flush();
        break;

    case MSGCODE_SET_DEVICE_TYPE:
        s_cfg.device_type = param;
        send_accepted_echo(param);
        break;

    case MSGCODE_GET_HDMI_VERSION:
        msg_begin(MSGCODE_GET_HDMI_VERSION);
        msg_put_escaped(s_cfg.hdmi_version);
        msg_end();
        wb_flush();
        break;

    case MSGCODE_SET_HDMI_VERSION:
        s_cfg.hdmi_version = param;
        send_accepted_echo(param);
        break;

    case MSGCODE_GET_OSD_NAME: {
        int name_len = strlen(s_cfg.osd_name);
        msg_begin(MSGCODE_GET_OSD_NAME);
        for (int i = 0; i < name_len; i++) {
            msg_put_escaped((uint8_t)s_cfg.osd_name[i]);
        }
        msg_end();
        wb_flush();
        break;
    }

    case MSGCODE_SET_OSD_NAME: {
        int name_len = len - 1;
        if (name_len > (int)(sizeof(s_cfg.osd_name) - 1))
            name_len = sizeof(s_cfg.osd_name) - 1;
        if (name_len > 0) {
            memcpy(s_cfg.osd_name, &msg[1], name_len);
            s_cfg.osd_name[name_len] = '\0';
            send_accepted_echo(msg[1]);
        } else {
            send_accepted();
        }
        break;
    }

    case MSGCODE_WRITE_EEPROM:
        settings_save();
        send_accepted();
        break;

    case MSGCODE_START_BOOTLOADER:
        send_accepted();
        platform_sleep_ms(100);
        platform_restart();
        break;

    case MSGCODE_SET_ACTIVE_SOURCE:
        send_accepted();
        break;

    default:
        PLAT_LOGW(TAG, "Unknown command 0x%02X", code);
        send_rejected();
        break;
    }
}

/* ------------------------------------------------------------------ */
/* Public: feed a byte from USB into the parser                         */
/* ------------------------------------------------------------------ */

void p8_rx_byte(uint8_t b) {
    if (b == P8_MSGSTART) {
        s_msg_len = 0;
        s_in_esc  = false;
        return;
    }
    if (b == P8_MSGEND) {
        if (s_msg_len > 0) handle_message(s_msg_buf, s_msg_len);
        s_msg_len = 0;
        s_in_esc  = false;
        return;
    }
    if (s_in_esc) {
        s_in_esc = false;
        b += P8_ESCOFFSET;         /* reverse the (byte - 3) encoding */
    } else if (b == P8_MSGESC) {
        s_in_esc = true;
        return;
    }
    if (s_msg_len < MSG_BUF_SIZE) {
        s_msg_buf[s_msg_len++] = b;
    }
}

/* ------------------------------------------------------------------ */
/* Init                                                                 */
/* ------------------------------------------------------------------ */

esp_err_t p8_protocol_init(void) {
    platform_mutex_init(&s_tx_mutex);
    platform_init();   /* opens NVS on ESP32-C6; no-op on RP2350 */

    settings_load();

    /*
     * Keep the CEC bus ACK mask at 0 until libCEC sends SET_ACK_MASK.
     * Pre-arming with the stored mask would cause firmware to ACK libCEC's
     * self-POLLs, making it think all playback addresses are occupied.
     */
    cec_bus_set_ack_mask(0);

    /*
     * Send firmware version unsolicited — real P8 adapters do this at startup.
     * Bytes may be lost before a USB host connects; that's fine, both libCEC
     * and the kernel driver query FIRMWARE_VERSION explicitly.
     */
    platform_mutex_lock(&s_tx_mutex);
    s_wbuf_len = 0;
    wb_put(P8_MSGSTART);
    wb_put(MSGCODE_FIRMWARE_VERSION);
    msg_put_escaped(P8_FIRMWARE_VERSION_MAJOR);
    msg_put_escaped(P8_FIRMWARE_VERSION_MINOR);
    wb_put(P8_MSGEND);
    wb_flush();

    PLAT_LOGI(TAG, "P8 protocol ready (fw v%d.%d)",
             P8_FIRMWARE_VERSION_MAJOR, P8_FIRMWARE_VERSION_MINOR);
    return ESP_OK;
}
