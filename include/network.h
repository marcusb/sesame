#ifndef _NETWORK_H
#define _NETWORK_H

void network_manager_task(void *const params);

typedef enum {
    NM_CMD_UNKNOWN = 0,
    NM_CMD_AP_MODE = 1,
} nm_cmd_t;

#endif
