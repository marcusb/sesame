#pragma once

void network_manager_task(void *params);

typedef enum {
    NM_CMD_UNKNOWN = 0,
    NM_CMD_AP_MODE = 1,
} nm_msg_type_t;

typedef struct {
    nm_msg_type_t type;
} nm_msg_t;
