#pragma once

#include <stdint.h>

#include "controller.h"

// fwd declaration
struct partition_entry;

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
    struct partition_entry* part;
    uint32_t flash_addr;
    uint32_t bytes_stored;
} ota_upd_state_t;

void ota_task(void*);

int ota_init(ota_upd_state_t* ota_state);
int ota_write_chunk(ota_upd_state_t* ota_state, const uint8_t* buf,
                    uint32_t len);
int ota_finish(ota_upd_state_t* ota_state);
void check_ota_test_image();
int ota_promote_image();
