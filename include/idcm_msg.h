#pragma once

#include <assert.h>
#include <stdint.h>

#define DCM_HEADER_BYTE 0x55

// must be larger than any message payload, to accommodate checksum byte
#define MAX_DCM_MSG_SIZE 20

typedef enum {
    DCM_MSG_0x04 = 0x04,
    DCM_MSG_DOOR_CMD = 0x10,
    DCM_MSG_ALERT_CMD = 0x11,
    DCM_MSG_AUDIO_CMD = 0x12,
    /** Periodic 14-byte door status report from PIC to MW300 */
    DCM_MSG_STATUS_UPDATE_14 = 0x14,
    DCM_MSG_DOOR_STATUS_REQUEST = 0x15,
    /** Extended 29-byte status update */
    DCM_MSG_DOOR_STATUS_UPDATE = 0x16,
    DCM_MSG_OPS_EVENT = 0x17,
    /** Factory diagnostic / self-test mode */
    DCM_MSG_FACTORY_TEST = 0x18,
    /** Direct GPIO & peripheral test command (PWM, light, buzzer, relay) */
    DCM_MSG_DIAG_OUTPUT_CMD = 0x1A,
    /** Combined visual and audio alert command */
    DCM_MSG_COMBINED_ALERT_CMD = 0x1B,
    DCM_MSG_DOOR_ACK = 0x90,
    DCM_MSG_ALERT_ACK = 0x91,
    DCM_MSG_AUDIO_ACK = 0x92,
    DCM_MSG_ACK_0x94 = 0x94,
    DCM_MSG_SENSOR_VERSION = 0x95,
    DCM_MSG_FACTORY_TEST_ACK = 0x98,
    DCM_MSG_DIAG_OUTPUT_ACK = 0x9A,
    DCM_MSG_COMBINED_ALERT_ACK = 0x9B,
} __attribute__((packed)) dcm_msg_type_t;

typedef enum {
    DCM_DOOR_STATE_CLOSED,
    DCM_DOOR_STATE_OPEN,  // "NOT CLOSED"
    DCM_DOOR_STATE_UNKNOWN,
    /** Factory test mode */
    DCM_DOOR_STATE_FTM
} __attribute__((packed)) door_open_state_t;

typedef enum {
    DCM_DOOR_DIR_DOWN,
    DCM_DOOR_DIR_UP,
    DCM_DOOR_DIR_STOPPED,
    DCM_DOOR_DIR_UNKNOWN
} __attribute__((packed)) door_direction_t;

typedef enum {
    OPS_EVT_UNUSED,
    OPS_NEW_LIMITS_DETECTED,
    OPS_POSITION_ADJUST,
    OPS_LIGHT_ON,
    OPS_LIGHT_ALERT_DONE,
    OPS_AUDIO_ALERT_DONE,
    OPS_MOTOR_START = 8,
    OPS_MOTOR_STOP = 9
} __attribute__((packed)) ops_event_t;

typedef struct {
    char zero[9];
} __attribute__((packed)) dcm_cmd_0x04_msg_t;
static_assert(sizeof(dcm_cmd_0x04_msg_t) == 9, "msg size");
static_assert(sizeof(dcm_cmd_0x04_msg_t) < MAX_DCM_MSG_SIZE, "msg size");

typedef struct {
    uint8_t val;       // 1 = open, 0 = close
    uint8_t reserved;  // always 0
} __attribute__((packed)) dcm_door_cmd_msg_t;
static_assert(sizeof(dcm_door_cmd_msg_t) == 2, "msg size");
static_assert(sizeof(dcm_door_cmd_msg_t) < MAX_DCM_MSG_SIZE, "msg size");

typedef struct {
    uint8_t val;
    // Duration in seconds, 5 for normal operation.
    uint8_t duration_s;
    uint8_t reserved;  // always 0
} __attribute__((packed)) dcm_alert_cmd_msg_t;
static_assert(sizeof(dcm_alert_cmd_msg_t) == 3, "msg size");
static_assert(sizeof(dcm_alert_cmd_msg_t) < MAX_DCM_MSG_SIZE, "msg size");

typedef struct {
    uint8_t val;
    // Duration of the tone in seconds. 0 during self-test (PIC ACKs without
    // audible output), 5 for normal operation.
    uint8_t duration_s;
    uint8_t reserved;  // always 0
} __attribute__((packed)) dcm_audio_cmd_msg_t;
static_assert(sizeof(dcm_audio_cmd_msg_t) == 3, "msg size");
static_assert(sizeof(dcm_audio_cmd_msg_t) < MAX_DCM_MSG_SIZE, "msg size");

typedef struct {
    uint32_t time;
    door_open_state_t state;
    door_direction_t direction;
    uint16_t pos;
    uint16_t up_limit;
    uint16_t down_limit;
} __attribute__((packed)) dcm_door_status_req_msg_t;
static_assert(sizeof(dcm_door_status_req_msg_t) == 12, "msg size");
static_assert(sizeof(dcm_door_status_req_msg_t) < MAX_DCM_MSG_SIZE, "msg size");

typedef struct {
    uint32_t time;
    door_open_state_t state;
    door_direction_t direction;
    uint16_t pos;
    uint16_t up_limit;
    uint16_t down_limit;
    // Motor load reading, likely an ADC value from the motor driver.
    // Zero when the motor is stopped; non-zero while moving. Consistently
    // higher when going UP (~36000–38000) than DOWN (~25000–33000), reflecting
    // the motor working against gravity when lifting. Zero on the first update
    // of each movement; settles by the second sample.
    uint16_t motor_current;
} __attribute__((packed)) dcm_door_status_update_msg_t;
static_assert(sizeof(dcm_door_status_update_msg_t) == 14, "msg size");
static_assert(sizeof(dcm_door_status_update_msg_t) < MAX_DCM_MSG_SIZE,
              "msg size");

typedef struct {
    uint8_t reserved;  // always 0
    door_open_state_t state;
    door_direction_t direction;
    uint8_t model_code;    // always 0x04; identifies DCM hardware variant
    uint8_t reserved2[2];  // always 0
    uint16_t pos;
    uint32_t time;
    uint8_t hw_caps;    // always 0x08; hardware capability bitfield
    uint8_t reserved3;  // always 0
    uint8_t sensor_restart_reason;
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    char suffix;
} __attribute__((packed)) dcm_sensor_version_msg_t;
static_assert(sizeof(dcm_sensor_version_msg_t) == 19, "msg size");
static_assert(sizeof(dcm_sensor_version_msg_t) < MAX_DCM_MSG_SIZE, "msg size");

typedef struct {
    uint32_t time;
    ops_event_t event;
    uint8_t reserved;  // always 0
    int16_t up_limit;
    int16_t down_limit;
} __attribute__((packed)) dcm_ops_event_msg_t;
static_assert(sizeof(dcm_ops_event_msg_t) == 10, "msg size");
static_assert(sizeof(dcm_ops_event_msg_t) < MAX_DCM_MSG_SIZE, "msg size");

typedef struct {
    uint8_t pwm_duty;
    uint8_t light_en;   // RA5 / alert light (1 = on, 0 = off)
    uint8_t buzzer_en;  // RA7 / buzzer (1 = on, 0 = off)
    uint8_t relay_en;   // RA1 / door trigger pulse (1 = on, 0 = off)
} __attribute__((packed)) dcm_diag_output_cmd_msg_t;
static_assert(sizeof(dcm_diag_output_cmd_msg_t) == 4, "msg size");
static_assert(sizeof(dcm_diag_output_cmd_msg_t) < MAX_DCM_MSG_SIZE, "msg size");

typedef struct {
    uint8_t status;
    uint8_t reserved;
} __attribute__((packed)) dcm_diag_output_ack_msg_t;
static_assert(sizeof(dcm_diag_output_ack_msg_t) == 2, "msg size");
static_assert(sizeof(dcm_diag_output_ack_msg_t) < MAX_DCM_MSG_SIZE, "msg size");

typedef struct {
    /** always 0x55 */
    uint8_t header;
    /** payload length */
    uint8_t len;
    /** sequence number ("token") */
    uint8_t seq;
    /** payload type */
    dcm_msg_type_t type;
    union {
        dcm_cmd_0x04_msg_t cmd_0x04;
        dcm_door_cmd_msg_t door_cmd;
        dcm_alert_cmd_msg_t alert_cmd;
        dcm_audio_cmd_msg_t audio_cmd;
        dcm_door_status_req_msg_t door_status_req;
        dcm_door_status_update_msg_t door_status;
        dcm_sensor_version_msg_t sensor_version;
        dcm_ops_event_msg_t ops_event;
        dcm_diag_output_cmd_msg_t diag_output_cmd;
        dcm_diag_output_ack_msg_t diag_output_ack;
        // Include a byte array, needs to be at least as big as the other
        // members! This is so that we can add a checksum byte.
        char buf[MAX_DCM_MSG_SIZE];
    } payload;
    // checksum byte follows payload
} __attribute__((packed)) dcm_msg_t;

/*
 * =============================================================================
 * PIC-to-PIC Inter-board Protocol & Hardware Signaling
 * =============================================================================
 *
 * The iDCM board PIC16LF15355 connects to the Main Board PIC16 via both
 * a dedicated serial link (UART2) and discrete hardware signaling lines:
 *
 * 1. Discrete Hardware Lines:
 *    - RA1 (Door Relay Pulse): Pulsed high for ~10 ms to trigger the door
 *      mechanism, electrically simulating a wall console pushbutton press.
 *    - RA5 (Alert Light Pulse): Driven as a 1.0 Hz square wave (500 ms ON /
 *      500 ms OFF) in response to DCM_MSG_ALERT_CMD (0x11). It directly pulses
 *      the main board opener light.
 *    - RA7 (Buzzer): On-board audio buzzer on the iDCM board.
 *    - RB0/RB1: Motor optical quadrature encoder inputs (CLCIN2/3 and IOC)
 *      for door position tracking and speed/stall monitoring.
 *
 * 2. UART2 Binary Protocol (115200 baud, 8-N-1 on pins RB3 TX, RB4 RX):
 *    Fixed 9-byte packet framing:
 *      [0x01 (SOH), 0x06 (LEN), <cmd>, <param>, <d0>, <d1>, <d2>, <d3>, 0x04 (EOT)]
 *
 *    Commands:
 *      - 0x08: Movement / status sync
 *      - 0x0C: Heartbeat polling query
 *      - 0x2E: Position calibration sync
 *      - 0x64: Up limit parameter sync
 *      - 0x65: Down limit parameter sync
 *      - 0xC8: Motor current threshold calibration
 *      - 0xD1: Motor load / operational status report
 *      - 0x03: Motor abort / stop command
 */
