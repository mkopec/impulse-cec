/*
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

#pragma once

/*
 * Pulse-Eight USB CEC adapter serial protocol — message codes and API.
 *
 * Wire format (libCEC 7.x, BOTH directions)
 * ==========================================
 * Every message is framed as:
 *   MSGSTART (0xFF)  cmd_byte  [param_bytes...]  MSGEND (0xFE)
 *
 * Bytes >= 0xFD appearing in cmd_byte or params are escaped:
 *   raw byte B  →  MSGESC (0xFD) followed by (B - 3)
 *     0xFD → 0xFD 0xFA
 *     0xFE → 0xFD 0xFB
 *     0xFF → 0xFD 0xFC
 *
 * Command byte format (applies to FRAME_START / FRAME_DATA messages only):
 *   bit 7 : EOM flag  (0x80) — set on the last CEC byte of a received frame
 *   bit 6 : ACK flag  (0x40) — set when the CEC frame was ACK'd on the bus
 *   bits 5:0 : message code (MSGCODE_*)
 *
 * For all non-frame messages the command byte IS the message code (no flags).
 *
 * Adapter → host responses:
 *   SET commands  → [0xFF, COMMAND_ACCEPTED(0x08), 0xFE]
 *   GET commands  → [0xFF, COMMAND_ACCEPTED(0x08), payload_bytes..., 0xFE]
 *   Errors        → [0xFF, COMMAND_REJECTED(0x09), 0xFE]
 *   PING          → [0xFF, PING(0x01), 0xFE]   (echo)
 *   FW version    → [0xFF, FIRMWARE_VERSION(0x15), major, minor, 0xFE]
 *   CEC frame byte→ [0xFF, FRAME_START/DATA | EOM | ACK, cec_byte, 0xFE]
 */

#include <stdint.h>
#include <stdbool.h>
#include "cec_bus.h"   /* also provides esp_err_t / gpio_num_t portability */

/* ------------------------------------------------------------------ */
/* Pulse-Eight protocol framing constants                               */
/* ------------------------------------------------------------------ */

#define P8_MSGSTART   0xFF   /* start of every message */
#define P8_MSGEND     0xFE   /* end of every message   */
#define P8_MSGESC     0xFD   /* escape prefix for bytes >= 0xFD */
#define P8_ESCOFFSET  3      /* escaped_byte = raw_byte - ESCOFFSET */

/* Flags used in the command byte of FRAME_START / FRAME_DATA messages */
#define P8_EOM_FLAG   0x80   /* bit 7: End of CEC message */
#define P8_ACK_FLAG   0x40   /* bit 6: CEC ACK received   */

/* Message codes (bits 5:0 of the command byte) */
#define MSGCODE_NOTHING                      0x00
#define MSGCODE_PING                         0x01
#define MSGCODE_TIMEOUT_ERROR                0x02
#define MSGCODE_HIGH_ERROR                   0x03
#define MSGCODE_LOW_ERROR                    0x04
#define MSGCODE_FRAME_START                  0x05
#define MSGCODE_FRAME_DATA                   0x06
#define MSGCODE_RECEIVE_FAILED               0x07
#define MSGCODE_COMMAND_ACCEPTED             0x08
#define MSGCODE_COMMAND_REJECTED             0x09
#define MSGCODE_SET_ACK_MASK                 0x0A
#define MSGCODE_TRANSMIT                     0x0B
#define MSGCODE_TRANSMIT_EOM                 0x0C
#define MSGCODE_TRANSMIT_IDLETIME            0x0D
#define MSGCODE_TRANSMIT_ACK_POLARITY        0x0E
#define MSGCODE_TRANSMIT_LINE_TIMEOUT        0x0F
#define MSGCODE_TRANSMIT_SUCCEEDED           0x10
#define MSGCODE_TRANSMIT_FAILED_LINE         0x11
#define MSGCODE_TRANSMIT_FAILED_ACK          0x12
#define MSGCODE_TRANSMIT_FAILED_TIMEOUT_DATA 0x13
#define MSGCODE_TRANSMIT_FAILED_TIMEOUT_LINE 0x14
#define MSGCODE_FIRMWARE_VERSION             0x15
#define MSGCODE_START_BOOTLOADER             0x16
#define MSGCODE_GET_BUILDDATE                0x17
#define MSGCODE_SET_CONTROLLED               0x18
#define MSGCODE_GET_AUTO_ENABLED             0x19
#define MSGCODE_SET_AUTO_ENABLED             0x1A
#define MSGCODE_GET_DEFAULT_LOGICAL_ADDRESS  0x1B
#define MSGCODE_SET_DEFAULT_LOGICAL_ADDRESS  0x1C
#define MSGCODE_GET_LOGICAL_ADDRESS_MASK     0x1D
#define MSGCODE_SET_LOGICAL_ADDRESS_MASK     0x1E
#define MSGCODE_GET_PHYSICAL_ADDRESS         0x1F
#define MSGCODE_SET_PHYSICAL_ADDRESS         0x20
#define MSGCODE_GET_DEVICE_TYPE              0x21
#define MSGCODE_SET_DEVICE_TYPE              0x22
#define MSGCODE_GET_HDMI_VERSION             0x23
#define MSGCODE_SET_HDMI_VERSION             0x24
#define MSGCODE_GET_OSD_NAME                 0x25
#define MSGCODE_SET_OSD_NAME                 0x26
#define MSGCODE_WRITE_EEPROM                 0x27
#define MSGCODE_GET_ADAPTER_TYPE             0x28
#define MSGCODE_SET_ACTIVE_SOURCE            0x29
#define MSGCODE_GET_AUTO_POWER_ON            0x2A
#define MSGCODE_SET_AUTO_POWER_ON            0x2B

/* Adapter type value */
#define ADAPTERTYPE_USBCEC  0x01

/* Firmware version (reported as two bytes: major, minor) */
#define P8_FIRMWARE_VERSION_MAJOR  0x00
#define P8_FIRMWARE_VERSION_MINOR  0x02

/* ------------------------------------------------------------------ */
/* API                                                                  */
/* ------------------------------------------------------------------ */

/**
 * Initialise the P8 protocol handler.
 * Must be called after cec_bus_init().
 */
esp_err_t p8_protocol_init(void);

/**
 * Feed a byte received from the USB CDC interface into the P8 parser.
 * Call this for every byte that arrives from the host.
 */
void p8_rx_byte(uint8_t b);

/**
 * Send a received CEC frame to the host in P8 format.
 * Called by cec_bus when a frame arrives.
 */
void p8_send_cec_frame(const cec_frame_t *frame);
