#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
// clang-format off
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/net/dhcpv4_server.h>
#include <zephyr/net/dhcpv6.h>
#include <zephyr/net/dns_resolve.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>
// clang-format on
#include <zephyr/random/random.h>

#include "config_manager.h"
#include "leds.h"
#include "network.h"

LOG_MODULE_REGISTER(network_manager, LOG_LEVEL_INF);

static bool is_sta_mode = false;
static struct k_work_delayable reconnect_work;
static struct net_mgmt_event_callback wifi_mgmt_cb;
static struct net_mgmt_event_callback ipv4_mgmt_cb;
static struct net_mgmt_event_callback ipv6_mgmt_cb;
static struct net_mgmt_event_callback dns_mgmt_cb;

static void reconnect_work_handler(struct k_work* work) {
    if (is_sta_mode) {
        network_manager_reconnect();
    }
}

static void log_existing_ipv6_addresses(struct net_if* iface) {
    if (iface && iface->config.ip.ipv6) {
        for (int i = 0; i < NET_IF_MAX_IPV6_ADDR; i++) {
            if (iface->config.ip.ipv6->unicast[i].is_used) {
                static char buf[NET_IPV6_ADDR_LEN];
                LOG_INF("IPv6 address: %s",
                        net_addr_ntop(
                            AF_INET6,
                            &iface->config.ip.ipv6->unicast[i].address.in6_addr,
                            buf, sizeof(buf)));
            }
        }
    }
}

static void wifi_mgmt_event_handler(struct net_mgmt_event_callback* cb,
                                    uint64_t mgmt_event, struct net_if* iface) {
    switch (mgmt_event) {
        case NET_EVENT_WIFI_AP_ENABLE_RESULT:
            set_wifi_led_pattern(LED_BLUE, LED_OFF, LED_BLUE, LED_OFF);
            break;
        case NET_EVENT_WIFI_AP_DISABLE_RESULT:
            set_wifi_led_pattern(LED_OFF, LED_OFF, LED_OFF, LED_OFF);
            break;
        case NET_EVENT_WIFI_CONNECT_RESULT: {
            k_work_cancel_delayable(&reconnect_work);
            set_wifi_led_pattern(LED_GREEN, LED_GREEN, LED_GREEN, LED_GREEN);
            net_dhcpv4_start(net_if_get_default());
            struct net_dhcpv6_params params = {.request_addr = false,
                                               .request_prefix = false};
            net_dhcpv6_start(net_if_get_default(), &params);

#if defined(CONFIG_NET_IPV6_ND) && defined(CONFIG_NET_NATIVE_IPV6)
            struct net_if* def_iface = net_if_get_default();
            if (def_iface && def_iface->config.ip.ipv6) {
                def_iface->config.ip.ipv6->rs_count = 0;
                net_if_start_rs(def_iface);
            }
#endif
            break;
        }
        case NET_EVENT_IPV4_ADDR_ADD: {
            if (cb->info) {
                static char buf[NET_IPV4_ADDR_LEN];
                LOG_INF("IPv4 address: %s",
                        net_addr_ntop(AF_INET, cb->info, buf, sizeof(buf)));
            }
            break;
        }
        case NET_EVENT_IPV6_ADDR_ADD: {
            if (cb->info) {
                static char buf[NET_IPV6_ADDR_LEN];
                LOG_INF("IPv6 address: %s",
                        net_addr_ntop(AF_INET6, cb->info, buf, sizeof(buf)));
            }
            break;
        }
        case NET_EVENT_IPV4_ROUTER_ADD: {
            if (cb->info) {
                static char buf[NET_IPV4_ADDR_LEN];
                LOG_INF("IPv4 gateway: %s",
                        net_addr_ntop(AF_INET, cb->info, buf, sizeof(buf)));
            }
            break;
        }
        case NET_EVENT_IPV6_ROUTER_ADD: {
            if (cb->info) {
                static struct in6_addr last_gw;
                if (memcmp(&last_gw, cb->info, sizeof(struct in6_addr)) != 0) {
                    memcpy(&last_gw, cb->info, sizeof(struct in6_addr));
                    static char buf[NET_IPV6_ADDR_LEN];
                    LOG_INF(
                        "IPv6 gateway: %s",
                        net_addr_ntop(AF_INET6, cb->info, buf, sizeof(buf)));
                }
            }
            break;
        }
        case NET_EVENT_DNS_SERVER_ADD: {
            if (cb->info) {
                struct sockaddr* addr = (struct sockaddr*)cb->info;
                if (addr->sa_family == AF_INET) {
                    static char buf[NET_IPV4_ADDR_LEN];
                    LOG_INF("DNS server: %s",
                            net_addr_ntop(AF_INET, &net_sin(addr)->sin_addr,
                                          buf, sizeof(buf)));
                } else if (addr->sa_family == AF_INET6) {
                    static char buf[NET_IPV6_ADDR_LEN];
                    LOG_INF("DNS server: %s",
                            net_addr_ntop(AF_INET6, &net_sin6(addr)->sin6_addr,
                                          buf, sizeof(buf)));
                }
            }
            break;
        }
        case NET_EVENT_IPV6_DAD_SUCCEED:
        case NET_EVENT_IPV6_DHCP_BOUND:
        case NET_EVENT_IPV4_DHCP_BOUND:
            break;
        case NET_EVENT_WIFI_DISCONNECT_RESULT:
            set_wifi_led_pattern(LED_GREEN, LED_OFF, LED_GREEN, LED_OFF);
            if (is_sta_mode) {
                // Reconnect after 5 seconds backoff
                k_work_reschedule(&reconnect_work, K_SECONDS(5));
            }
            break;
        default:
            break;
    }
}

void network_manager_init(void) {
    k_work_init_delayable(&reconnect_work, reconnect_work_handler);

    net_mgmt_init_event_callback(
        &wifi_mgmt_cb, wifi_mgmt_event_handler,
        NET_EVENT_WIFI_AP_ENABLE_RESULT | NET_EVENT_WIFI_AP_DISABLE_RESULT |
            NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT);
    net_mgmt_add_event_callback(&wifi_mgmt_cb);

    net_mgmt_init_event_callback(&ipv4_mgmt_cb, wifi_mgmt_event_handler,
                                 NET_EVENT_IPV4_ADDR_ADD |
                                     NET_EVENT_IPV4_ROUTER_ADD |
                                     NET_EVENT_IPV4_DHCP_BOUND);
    net_mgmt_add_event_callback(&ipv4_mgmt_cb);

    net_mgmt_init_event_callback(
        &ipv6_mgmt_cb, wifi_mgmt_event_handler,
        NET_EVENT_IPV6_ADDR_ADD | NET_EVENT_IPV6_ROUTER_ADD |
            NET_EVENT_IPV6_DAD_SUCCEED | NET_EVENT_IPV6_DHCP_BOUND);
    net_mgmt_add_event_callback(&ipv6_mgmt_cb);

    net_mgmt_init_event_callback(&dns_mgmt_cb, wifi_mgmt_event_handler,
                                 NET_EVENT_DNS_SERVER_ADD);
    net_mgmt_add_event_callback(&dns_mgmt_cb);

    log_existing_ipv6_addresses(net_if_get_default());
}

void start_ap(void) {
    is_sta_mode = false;
    k_work_cancel_delayable(&reconnect_work);

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
    is_sta_mode = true;
    k_work_cancel_delayable(&reconnect_work);

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
