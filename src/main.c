#include <string.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/net/dhcpv4_server.h>
#include <zephyr/net/http/server.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_if.h>
// clang-format off
#include <zephyr/net/dhcpv4.h>
// clang-format on
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>

#include "config_manager.h"
#include "controller.h"
#include "leds.h"
#include "mflash_drv.h"
#include "network.h"

K_MSGQ_DEFINE(ctrl_queue, sizeof(ctrl_msg_t), 8, 4);

static const struct gpio_dt_spec wifi_button =
    GPIO_DT_SPEC_GET(DT_NODELABEL(sw_wifi), gpios);

static struct net_mgmt_event_callback wifi_mgmt_cb;
static const struct device* const wdt = DEVICE_DT_GET(DT_NODELABEL(wdt0));
static int wdt_channel_id = -1;

static struct k_work_delayable reconnect_work;
static bool is_sta_mode = false;

static void reconnect_work_handler(struct k_work* work) {
    if (is_sta_mode) {
        network_manager_reconnect();
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
        case NET_EVENT_WIFI_CONNECT_RESULT:
            set_wifi_led_pattern(LED_GREEN, LED_GREEN, LED_GREEN, LED_GREEN);
            net_dhcpv4_start(net_if_get_default());
            break;
        case NET_EVENT_IPV4_ADDR_ADD: {
            char buf[NET_IPV4_ADDR_LEN];
            LOG_INF("WiFi DHCP success! IP address added: %s",
                    net_addr_ntop(
                        AF_INET,
                        &iface->config.ip.ipv4->unicast[0].ipv4.address.in_addr,
                        buf, sizeof(buf)));
            break;
        }
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

    if (load_config() == 0) {
        LOG_INF("Config loaded successfully");
    } else {
        LOG_WRN("Failed to load config, using defaults");
    }

    if (app_config.has_network_config &&
        strlen(app_config.network_config.ssid) > 0) {
        is_sta_mode = true;
        start_sta();
    } else {
        is_sta_mode = false;
        start_ap();
    }

    net_mgmt_init_event_callback(
        &wifi_mgmt_cb, wifi_mgmt_event_handler,
        NET_EVENT_WIFI_AP_ENABLE_RESULT | NET_EVENT_WIFI_AP_DISABLE_RESULT |
            NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT);
    net_mgmt_add_event_callback(&wifi_mgmt_cb);

    static struct net_mgmt_event_callback ipv4_mgmt_cb;
    net_mgmt_init_event_callback(&ipv4_mgmt_cb, wifi_mgmt_event_handler,
                                 NET_EVENT_IPV4_ADDR_ADD);
    net_mgmt_add_event_callback(&ipv4_mgmt_cb);

    k_work_init_delayable(&reconnect_work, reconnect_work_handler);

    http_server_start();

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
        int get_ret = k_msgq_get(&ctrl_queue, &msg, K_MSEC(1000));
        if (get_ret == 0) {
            switch (msg.type) {
                case CTRL_MSG_WIFI_BUTTON:
                    is_sta_mode = false;
                    k_work_cancel_delayable(&reconnect_work);
                    start_ap();
                    break;
                case CTRL_MSG_WIFI_CONFIG:
                    app_config.network_config = msg.msg.network_cfg;
                    app_config.has_network_config = true;
                    if (save_network_config() == 0) {
                        k_msleep(1000);
                        sys_reboot(SYS_REBOOT_COLD);
                    } else {
                        LOG_ERR(
                            "Failed to save network config, aborting reboot");
                    }
                    break;
                case CTRL_MSG_MQTT_CONFIG:
                    app_config.mqtt_config = msg.msg.mqtt_cfg;
                    app_config.has_mqtt_config = true;
                    if (save_mqtt_config() == 0) {
                        k_msleep(1000);
                        sys_reboot(SYS_REBOOT_COLD);
                    } else {
                        LOG_ERR("Failed to save mqtt config, aborting reboot");
                    }
                    break;
                case CTRL_MSG_LOGGING_CONFIG:
                    app_config.logging_config = msg.msg.logging_cfg;
                    app_config.has_logging_config = true;
                    if (save_logging_config() == 0) {
                        k_msleep(1000);
                        sys_reboot(SYS_REBOOT_COLD);
                    } else {
                        LOG_ERR(
                            "Failed to save logging config, aborting reboot");
                    }
                    break;
                case CTRL_MSG_RESTART:
                    LOG_INF("Restarting system...");
                    k_msleep(500);
                    sys_reboot(SYS_REBOOT_COLD);
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
