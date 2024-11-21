#ifndef _DHCP_H
#define _DHCP_H

#include <stdint.h>

void dhcpd_task(void *params);

typedef struct {
    uint32_t ip_addr;
    uint32_t netmask;
} dhcp_task_params_t;

#endif
