#ifndef _IDCM_MSG_H
#define _IDCM_MSG_H

#include <stdint.h>

typedef enum {
    DCM_MSG_DOOR_STATUS_REQUEST = 0x15,
    DCM_MSG_DOOR_STATUS_RESPONSE = 0x16,
    DCM_MSG_OPS_EVENT = 0x17,
    DCM_AUDIO_RESPONSE = 0x92,
    DCM_MSG_SENSOR_VERSION = 0x95,
} __attribute__((packed)) dcm_msg_type_t;

typedef enum {
    DOORSTATE_CLOSED,
    DOORSTATE_OPEN,
    DOORSTATE_UNKNOWN,
    /** Factory test mode */
    DOORSTATE_FTM
} __attribute__((packed)) door_state_t;

typedef enum {
    DOOR_DIR_DOWN,
    DOOR_DIR_UP,
    DOOR_DIR_STOPPED,
    DOOR_DIR_UNKNOWN
} __attribute__((packed)) door_direction_t;

typedef enum {
    OPS_EVT_UNUSED,
    OPS_NEW_LIMITS_DETECTED,
    OPS_POSITION_ADJUST,
    OPS_LIGHT_ON,
    OPS_MOVE_OPEN,  //??
    OPS_MOVE_CLOSE,
    OPS_MOTOR_START = 8,
    OPS_MOTOR_STOP = 9
} __attribute__((packed)) ops_event_t;

typedef struct {
    uint32_t time;
    door_state_t state;
    door_direction_t direction;
    int16_t pos;
    int16_t up_limit;
    int16_t down_limit;

} __attribute__((packed)) dcm_door_status_msg_t;
_Static_assert(sizeof(dcm_door_status_msg_t) == 12, "msg size");

typedef struct {
    char unk[8];
    uint32_t time;
    char unk2[3];
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
    char suffix;
} __attribute__((packed)) dcm_sensor_version_msg_t;
_Static_assert(sizeof(dcm_sensor_version_msg_t) == 19, "msg size");

typedef struct {
    uint32_t time;
    ops_event_t event;
    char unk;
    uint16_t v1;
    uint16_t v2;
} __attribute__((packed)) dcm_ops_event_msg_t;
_Static_assert(sizeof(dcm_ops_event_msg_t) == 10, "msg size");

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
        dcm_door_status_msg_t door_status;
        dcm_sensor_version_msg_t sensor_version;
        dcm_ops_event_msg_t ops_event;
    } payload;
    // checksum byte follows payload
} __attribute__((packed)) dcm_msg_t;

#endif
