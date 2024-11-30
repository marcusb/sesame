#pragma once

#include "controller.h"
#include "psm-v2.h"

#define MAX_SSID_LEN 32

void network_manager_task(void* params);

typedef struct {
    void *nm_queue;
    psm_hnd_t *psm_hnd;
} nm_task_params_t;

typedef enum {
    NM_CMD_UNKNOWN = 0,
    NM_CMD_AP_MODE = 1,
    NM_CMD_WIFI_CONFIG = 2,
} nm_msg_type_t;

typedef struct {
    nm_msg_type_t type;
    union {
        wifi_cfg_msg_t wifi_cfg;
    } msg;
} nm_msg_t;

struct xNetworkEndPoint;
void print_ip_config(struct xNetworkEndPoint* endpoint);
