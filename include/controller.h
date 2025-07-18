#pragma once

#include "FreeRTOS.h"
#include "api.pb.h"
#include "app_config.pb.h"
#include "idcm_msg.h"

typedef enum {
    CTRL_MSG_UNKNOWN,
    CTRL_MSG_OTA_UPGRADE,
    CTRL_MSG_OTA_PROMOTE,

    CTRL_MSG_DOOR_CONTROL,
    CTRL_MSG_DOOR_STATE_UPDATE,
    CTRL_MSG_WIFI_BUTTON,
    CTRL_MSG_OTA_BUTTON,
    CTRL_MSG_WIFI_CONFIG,
    CTRL_MSG_MQTT_CONFIG,
    CTRL_MSG_LOGGING_CONFIG,
} ctrl_msg_type_t;

typedef enum { DOOR_CMD_UNKNOWN, DOOR_CMD_OPEN, DOOR_CMD_CLOSE } door_cmd_t;

typedef struct {
    door_cmd_t command;
} door_control_msg_t;

typedef struct {
    door_open_state_t state;
    door_direction_t direction;
    int pos;
} door_state_msg_t;

typedef struct {
    ctrl_msg_type_t type;

    union {
        FirmwareUpgradeFetchRequest ota_upgrade;
        door_control_msg_t door_control;
        door_state_msg_t door_state;
        NetworkConfig network_cfg;
        MqttConfig mqtt_cfg;
        LoggingConfig logging_cfg;
    } msg;
} ctrl_msg_t;

typedef struct QueueDefinition* QueueHandle_t;
extern QueueHandle_t ctrl_queue;

void reboot(void);
