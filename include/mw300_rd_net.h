#pragma once

#include <stdint.h>

struct xNetworkInterface;
struct xNetworkInterface *mw300_new_netif_desc(
    uint8_t if_type, struct xNetworkInterface *netif);

void notify_dhcp_configured();
