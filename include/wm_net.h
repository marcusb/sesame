#pragma once

#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"
#include "wlan.h"

#define htonl(x) FreeRTOS_htonl(x)

void net_wlan_init(void);
int net_get_if_addr(struct wlan_ip_config *addr, void *intrfc_handle);
void net_configure_dns(struct wlan_ip_config *ip, enum wlan_bss_role role);
void net_interface_down(void *intrfc_handle);
void net_interface_dhcp_stop(void *intrfc_handle);
int net_configure_address(struct wlan_ip_config *addr, void *intrfc_handle);
void *net_get_mlan_handle();
void *net_get_uap_handle(void);
int wrapper_wlan_handle_amsdu_rx_packet(const t_u8 *rcvdata,
                                        const t_u16 datalen);
