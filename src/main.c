#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/sys/printk.h>

static void start_ap(void) {
    struct net_if* iface = net_if_get_default();
    if (!iface) {
        printk("No default network interface\n");
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

    printk("Starting WiFi AP...\n");
    if (net_mgmt(NET_REQUEST_WIFI_AP_ENABLE, iface, &ap_params,
                 sizeof(ap_params))) {
        printk("Failed to start AP\n");
    } else {
        printk("AP started successfully\n");
    }
}

void main(void) {
    const struct device* const wdt = DEVICE_DT_GET(DT_NODELABEL(wdt0));

    if (!device_is_ready(wdt)) {
        printk("Watchdog device not ready\n");
        return;
    }

    struct wdt_timeout_cfg wdt_config = {
        .window.min = 0U,
        .window.max = 2000U,
        .callback = NULL,
        .flags = WDT_FLAG_RESET_SOC,
    };

    int wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
    if (wdt_channel_id < 0) {
        printk("Watchdog install error\n");
        return;
    }

    wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);

    start_ap();

    while (1) {
        wdt_feed(wdt, wdt_channel_id);
        k_sleep(K_MSEC(1000));
    }
}
