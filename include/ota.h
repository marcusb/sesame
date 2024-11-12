#ifndef _OTA_H
#define _OTA_H

void ota_task(void *);

typedef enum {
    OTA_CMD_UNKNOWN = 0,
    OTA_CMD_UPGRADE = 1,
} ota_cmd_t;

#endif
