#pragma once

void network_manager_task(void* params);

struct xNetworkEndPoint;
void print_ip_config(struct xNetworkEndPoint* endpoint);

typedef enum {
    NM_CMD_UNKNOWN = 0,
    NM_CMD_AP_MODE = 1,
} nm_cmd_t;
