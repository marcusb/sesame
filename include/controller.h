#pragma once

#include <zephyr/kernel.h>

#if __has_include("api.pb.h")
#include "api.pb.h"
#include "app_config.pb.h"
#define HAVE_PROTOBUF_MSGS 1
#endif
#include "idcm_msg.h"

typedef enum {
    CTRL_MSG_UNKNOWN,
    CTRL_MSG_OTA_UPGRADE,
    CTRL_MSG_OTA_PROMOTE,

    CTRL_MSG_DOOR_CONTROL,
    CTRL_MSG_DOOR_STATE_UPDATE,
    CTRL_MSG_LIGHT_CONTROL,
    CTRL_MSG_LIGHT_STATE_UPDATE,
    CTRL_MSG_WIFI_BUTTON,
    CTRL_MSG_OTA_BUTTON,
    CTRL_MSG_WIFI_CONFIG,
    CTRL_MSG_MQTT_CONFIG,
    CTRL_MSG_LOGGING_CONFIG,
    CTRL_MSG_RESTART,
} ctrl_msg_type_t;

typedef enum {
    DOOR_CMD_UNKNOWN,
    DOOR_CMD_OPEN,
    DOOR_CMD_CLOSE,
    DOOR_CMD_STOP
} door_cmd_t;

typedef struct {
    door_cmd_t command;
} door_control_msg_t;

typedef struct {
    bool on;
} light_control_msg_t;

typedef struct {
    bool on;
} light_state_msg_t;

typedef struct {
    door_open_state_t state;
    door_direction_t direction;
    int pos;
} door_state_msg_t;

typedef struct {
    ctrl_msg_type_t type;

    union {
#ifdef HAVE_PROTOBUF_MSGS
        FirmwareUpgradeFetchRequest ota_upgrade;
#endif
        door_control_msg_t door_control;
        door_state_msg_t door_state;
        light_control_msg_t light_control;
        light_state_msg_t light_state;
#ifdef HAVE_PROTOBUF_MSGS
        NetworkConfig network_cfg;
        MqttConfig mqtt_cfg;
        LoggingConfig logging_cfg;
#endif
    } msg;
} ctrl_msg_t;

extern struct k_msgq ctrl_queue;

void reboot(void);
