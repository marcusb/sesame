#pragma once

#include <stdbool.h>
#include "FreeRTOS.h"
#include "NetworkInterface.h"

void board_init(void);
void configure_netif(void);
void create_board_tasks(void);
void reboot(void);
void board_refresh_watchdog(void);

void notify_board_dhcp_configured(void);
void notify_board_ipv6_addr_change(void);
bool is_sta_iface(NetworkInterface_t* iface);
bool is_uap_iface(NetworkInterface_t* iface);
