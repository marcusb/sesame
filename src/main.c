#include <string.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/net/dhcpv4_server.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/sys/printk.h>

#include "controller.h"
#include "leds.h"

K_MSGQ_DEFINE(ctrl_queue, sizeof(ctrl_msg_t), 8, 4);

static const struct gpio_dt_spec wifi_button =
    GPIO_DT_SPEC_GET(DT_NODELABEL(sw_wifi), gpios);

static struct net_mgmt_event_callback wifi_mgmt_cb;
static const struct device* const wdt = DEVICE_DT_GET(DT_NODELABEL(wdt0));
static int wdt_channel_id = -1;

static void wifi_mgmt_event_handler(struct net_mgmt_event_callback* cb,
                                    uint64_t mgmt_event, struct net_if* iface) {
    switch (mgmt_event) {
        case NET_EVENT_WIFI_AP_ENABLE_RESULT:
            set_wifi_led_pattern(LED_BLUE, LED_OFF, LED_BLUE, LED_OFF);
            printk("WiFi AP enabled\n");
            break;
        case NET_EVENT_WIFI_AP_DISABLE_RESULT:
            set_wifi_led_pattern(LED_OFF, LED_OFF, LED_OFF, LED_OFF);
            printk("WiFi AP disabled\n");
            break;
        case NET_EVENT_WIFI_CONNECT_RESULT:
            set_wifi_led_pattern(LED_GREEN, LED_GREEN, LED_GREEN, LED_GREEN);
            printk("WiFi Connected\n");
            break;
        case NET_EVENT_WIFI_DISCONNECT_RESULT:
            set_wifi_led_pattern(LED_GREEN, LED_OFF, LED_GREEN, LED_OFF);
            printk("WiFi Disconnected\n");
            break;
        default:
            break;
    }
}

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
        char ip_str[INET_ADDRSTRLEN];
        char mask_str[INET_ADDRSTRLEN];
        net_addr_ntop(AF_INET, &ap_ip, ip_str, sizeof(ip_str));
        net_addr_ntop(AF_INET, &ap_mask, mask_str, sizeof(mask_str));
        printk("AP started successfully. IP: %s, Netmask: %s\n", ip_str,
               mask_str);
    }

    struct in_addr dhcp_base_ip;
    net_addr_pton(AF_INET, "192.168.4.2", &dhcp_base_ip);
    if (net_dhcpv4_server_start(iface, &dhcp_base_ip) < 0) {
        printk("Failed to start DHCPv4 server\n");
    } else {
        printk("DHCPv4 server started\n");
    }
}

static void init_watchdog() {
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

    wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
    if (wdt_channel_id < 0) {
        printk("Watchdog install error\n");
        return;
    }

    wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
}

static struct gpio_callback wifi_button_cb_data;

static void wifi_button_pressed(const struct device* dev,
                                struct gpio_callback* cb, uint32_t pins) {
    ctrl_msg_t msg = {.type = CTRL_MSG_WIFI_BUTTON};
    k_msgq_put(&ctrl_queue, &msg, K_NO_WAIT);
}

void main(void) {
    leds_init();

    net_mgmt_init_event_callback(
        &wifi_mgmt_cb, wifi_mgmt_event_handler,
        NET_EVENT_WIFI_AP_ENABLE_RESULT | NET_EVENT_WIFI_AP_DISABLE_RESULT |
            NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT);
    net_mgmt_add_event_callback(&wifi_mgmt_cb);

    init_watchdog();
    set_ota_led_pattern(LED_GREEN, LED_GREEN, LED_OFF, LED_OFF);

    if (gpio_is_ready_dt(&wifi_button)) {
        gpio_pin_configure_dt(&wifi_button, GPIO_INPUT | GPIO_PULL_UP);
        gpio_pin_interrupt_configure_dt(&wifi_button, GPIO_INT_EDGE_TO_ACTIVE);
        gpio_init_callback(&wifi_button_cb_data, wifi_button_pressed,
                           BIT(wifi_button.pin));
        gpio_add_callback(wifi_button.port, &wifi_button_cb_data);
    }

    while (1) {
        ctrl_msg_t msg;
        if (k_msgq_get(&ctrl_queue, &msg, K_MSEC(1000)) == 0) {
            switch (msg.type) {
                case CTRL_MSG_WIFI_BUTTON:
                    printk("Starting AP mode from WIFI button...\n");
                    start_ap();
                    break;
                default:
                    break;
            }
        }

        if (wdt_channel_id >= 0) {
            wdt_feed(wdt, wdt_channel_id);
        }
    }
}
