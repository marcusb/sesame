#pragma once

#include <stdint.h>

void dhcpd_task(void *const params);

typedef struct xNetworkEndPoint NetworkEndPoint_t;

typedef struct {
    NetworkEndPoint_t *endpoint;
} dhcp_task_params_t;
