#pragma once

#include <stdint.h>

#include "controller.h"

#include <zephyr/dfu/flash_img.h>

typedef enum {
    OTA_CMD_UNKNOWN = 0,
    OTA_CMD_UPGRADE = 1,
    OTA_CMD_PROMOTE = 2,
} ota_cmd_t;

typedef struct {
    ota_cmd_t cmd;
    union {
        FirmwareUpgradeFetchRequest upgrade_msg;
    } msg;
} ota_msg_t;

typedef struct {
    struct flash_img_context ctx;
} ota_upd_state_t;

typedef enum {
    OTA_STATUS_NONE = 0,
    OTA_STATUS_UPLOADED,
    OTA_STATUS_TESTING,
} ota_status_t;
extern ota_status_t ota_status;

int ota_init(ota_upd_state_t* ota_state);
int ota_write_chunk(ota_upd_state_t* ota_state, const uint8_t* buf,
                    uint32_t len);
int ota_finish(ota_upd_state_t* ota_state);
void check_ota_test_image();
int ota_promote_image();
void ota_client_start(const FirmwareUpgradeFetchRequest* req);
