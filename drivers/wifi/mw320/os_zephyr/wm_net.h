#ifndef _ZEPHYR_WM_NET_H_
#define _ZEPHYR_WM_NET_H_

#include <zephyr/sys/byteorder.h>
#include <stdint.h>

#define htonl(x) sys_cpu_to_be32(x)

#define IP6_ADDR_PREFERRED 1
#define IP6_ADDR_OTHER 2

struct wlan_ip_config;
enum wlan_bss_role;

void net_wlan_init(void);
int net_get_if_addr(struct wlan_ip_config *addr, void *intrfc_handle);
int net_get_if_ipv6_addr(struct wlan_ip_config *addr, void *intrfc_handle);
void net_configure_dns(struct wlan_ip_config *ip, enum wlan_bss_role role);
void net_interface_down(void *intrfc_handle);
void net_interface_dhcp_stop(void *intrfc_handle);
int net_configure_address(struct wlan_ip_config *addr, void *intrfc_handle);
void *net_get_mlan_handle(void);
void *net_get_uap_handle(void);
int wrapper_wlan_handle_amsdu_rx_packet(const uint8_t *rcvdata,
                                        const uint16_t datalen);

#endif /* _ZEPHYR_WM_NET_H_ */
