#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/dhcpv4_server.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>

#include "config_manager.h"
#include "network.h"

LOG_MODULE_REGISTER(network_manager, LOG_LEVEL_INF);

void start_ap(void) {
    struct net_if* iface = net_if_get_default();
    if (!iface) {
        LOG_ERR("No default network interface");
        return;
    }

    struct in_addr ap_ip;
    net_addr_pton(AF_INET, "192.168.4.1", &ap_ip);

    struct in_addr ap_mask;
    net_addr_pton(AF_INET, "255.255.255.0", &ap_mask);

    net_if_ipv4_addr_add(iface, &ap_ip, NET_ADDR_MANUAL, 0);
    net_if_ipv4_set_netmask_by_addr(iface, &ap_ip, &ap_mask);

    struct wifi_connect_req_params ap_params = {0};
    ap_params.ssid = (uint8_t*)"sesame";
    ap_params.ssid_length = strlen("sesame");
    ap_params.channel = 6;
    ap_params.security = WIFI_SECURITY_TYPE_NONE;

    LOG_INF("Starting WiFi AP...");
    if (net_mgmt(NET_REQUEST_WIFI_AP_ENABLE, iface, &ap_params,
                 sizeof(ap_params))) {
        LOG_ERR("Failed to start AP");
    } else {
        char ip_str[INET_ADDRSTRLEN];
        char mask_str[INET_ADDRSTRLEN];
        net_addr_ntop(AF_INET, &ap_ip, ip_str, sizeof(ip_str));
        net_addr_ntop(AF_INET, &ap_mask, mask_str, sizeof(mask_str));
        LOG_INF("AP started successfully. IP: %s, Netmask: %s", ip_str,
                mask_str);
    }

    struct in_addr dhcp_base_ip;
    net_addr_pton(AF_INET, "192.168.4.2", &dhcp_base_ip);
    if (net_dhcpv4_server_start(iface, &dhcp_base_ip) < 0) {
        LOG_ERR("Failed to start DHCPv4 server");
    } else {
        LOG_INF("DHCPv4 server started");
    }
}

void start_sta(void) {
    struct net_if* iface = net_if_get_default();
    if (!iface) {
        LOG_ERR("No default network interface");
        return;
    }

    struct wifi_connect_req_params sta_params = {0};
    sta_params.ssid = (uint8_t*)app_config.network_config.ssid;
    sta_params.ssid_length = strlen(app_config.network_config.ssid);

    if (app_config.network_config.security > 0) {
        sta_params.psk = (uint8_t*)app_config.network_config.password;
        sta_params.psk_length = strlen(app_config.network_config.password);
        sta_params.security =
            WIFI_SECURITY_TYPE_PSK;  // Map accordingly if needed
    } else {
        sta_params.security = WIFI_SECURITY_TYPE_NONE;
    }

    // We want DHCP for STA mode, Zephyr handles DHCP automatically if
    // configured on the interface Note: Zephyr requires DHCP client to be
    // enabled in prj.conf (CONFIG_NET_DHCPV4=y) We start the DHCP client when
    // the interface goes up, or it might be auto-started.

    LOG_INF("Starting WiFi STA connection to %s...",
            app_config.network_config.ssid);
    if (net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &sta_params,
                 sizeof(sta_params))) {
        LOG_ERR("Failed to request STA connect");
    }
}

void network_manager_reconnect(void) {
    LOG_INF("Attempting to reconnect WiFi STA...");
    start_sta();
}
