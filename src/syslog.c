#include "syslog.h"

#include <stdbool.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_backend_net.h>
#include <zephyr/net/dns_resolve.h>
#include <zephyr/net/net_ip.h>

#include "app_config.pb.h"
#include "config_manager.h"
#include "network.h"

LOG_MODULE_REGISTER(syslog, LOG_LEVEL_INF);

static struct k_work_delayable retry_work;
static bool backend_active;

static void set_log_backend(const struct net_sockaddr* sa, const char* host) {
    if (log_backend_net_set_ip(sa)) {
        k_work_cancel_delayable(&retry_work);
        log_backend_net_start();
        backend_active = true;
        LOG_INF("Syslog host set: %s", host);
    }
}

static void dns_resolve_cb(enum dns_resolve_status status,
                           struct dns_addrinfo* info, void* user_data) {
    const SyslogConfig* scfg = &app_config.logging_config.syslog_config;
    const char* query = (const char*)user_data;

    if (backend_active) {
        return;
    }

    switch (status) {
        case DNS_EAI_CANCELED:
            return;
        case DNS_EAI_NODATA:
            LOG_DBG("No data for DNS query: %s", query);
            return;
        case DNS_EAI_ALLDONE:
            LOG_DBG("DNS resolving finished");
            return;
        case DNS_EAI_INPROGRESS:
            break;
        case DNS_EAI_AGAIN:
            LOG_DBG("DNS lookup temp fail, retrying after backoff: %s", query);
            k_work_reschedule(&retry_work, K_SECONDS(5));
            break;
        default:
            LOG_DBG("DNS lookup status (%d): %s", status, query);
            return;
    }

    if (!info) {
        return;
    }

    char addr_str[NET_IPV6_ADDR_LEN];
    uint16_t port = scfg->syslog_port ? scfg->syslog_port : 514;
    struct net_sockaddr sa;
    memcpy(&sa, &info->ai_addr, sizeof(sa));
    if (sa.sa_family == AF_INET) {
        LOG_INF("%s IPv4 address: %s", query,
                net_addr_ntop(info->ai_family, &net_sin(&sa)->sin_addr,
                              addr_str, sizeof(addr_str)));
        net_sin(&sa)->sin_port = net_htons(port);
    } else if (sa.sa_family == AF_INET6) {
        LOG_INF("%s IPv6 address: %s", query,
                net_addr_ntop(info->ai_family, &net_sin6(&sa)->sin6_addr,
                              addr_str, sizeof(addr_str)));
        net_sin6(&sa)->sin6_port = net_htons(port);
    }
    set_log_backend(&sa, query);
}

static void retry_work_handler(struct k_work* work) {
    ARG_UNUSED(work);

    const SyslogConfig* scfg = &app_config.logging_config.syslog_config;

    if (!network_is_up()) {
        k_work_reschedule(&retry_work, K_SECONDS(5));
        return;
    }

    int ret_a =
        dns_get_addr_info(scfg->syslog_host, DNS_QUERY_TYPE_A, NULL,
                          dns_resolve_cb, (void*)scfg->syslog_host, 10000);
    int ret_aaaa =
        dns_get_addr_info(scfg->syslog_host, DNS_QUERY_TYPE_AAAA, NULL,
                          dns_resolve_cb, (void*)scfg->syslog_host, 10000);
    if (ret_a < 0 && ret_aaaa < 0) {
        LOG_WRN("DNS resolve failed to send for %s, retrying in 5s",
                scfg->syslog_host);
        k_work_reschedule(&retry_work, K_SECONDS(5));
    }
}

void syslog_init(void) {
    const SyslogConfig* scfg = &app_config.logging_config.syslog_config;
    if (backend_active || !scfg->enabled || strlen(scfg->syslog_host) == 0) {
        return;
    }

    k_work_init_delayable(&retry_work, retry_work_handler);

    struct net_sockaddr sa = {0};
    if (net_ipaddr_parse(scfg->syslog_host, strlen(scfg->syslog_host), &sa)) {
        uint16_t port = scfg->syslog_port ? scfg->syslog_port : 514;
        if (sa.sa_family == AF_INET) {
            net_sin(&sa)->sin_port = net_htons(port);
        } else if (sa.sa_family == AF_INET6) {
            net_sin6(&sa)->sin6_port = net_htons(port);
        }

        set_log_backend(&sa, scfg->syslog_host);
        return;
    }

    k_work_reschedule(&retry_work, K_USEC(0));
}
