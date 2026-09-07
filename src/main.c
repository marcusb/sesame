#include <string.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#include <zephyr/app_version.h>
#include <zephyr/device.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/net/http/server.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>

#include "config_manager.h"
#include "controller.h"
#include "leds.h"
#include "matter_endpoints.h"
#ifdef CONFIG_SOC_88MW320
#include "mflash_drv.h"
#endif
#include "mqtt.h"
#ifdef CONFIG_SOC_88MW320
#include "mw_watchdog.h"
#endif
#include "network.h"
#ifdef CONFIG_BOOTLOADER_MCUBOOT
#include "ota.h"
#endif
#include "pic_uart.h"
#include "sesame_syslog.h"

K_MSGQ_DEFINE(ctrl_queue, sizeof(ctrl_msg_t), 8, 4);

#if DT_NODE_EXISTS(DT_NODELABEL(sw_wifi))
static const struct gpio_dt_spec wifi_button =
    GPIO_DT_SPEC_GET(DT_NODELABEL(sw_wifi), gpios);
#endif

extern char __sram1_bss_start[];
extern char __sram1_bss_end[];
#ifdef CONFIG_SOC_88MW320
static int zero_sram1_bss(void) {
    memset(__sram1_bss_start, 0, __sram1_bss_end - __sram1_bss_start);
    return 0;
}
SYS_INIT(zero_sram1_bss, PRE_KERNEL_1, 0);
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(sw_wifi))
static struct gpio_callback wifi_button_cb_data;

static void wifi_button_pressed(const struct device* dev,
                                struct gpio_callback* cb, uint32_t pins) {
    ctrl_msg_t msg = {.type = CTRL_MSG_WIFI_BUTTON};
    k_msgq_put(&ctrl_queue, &msg, K_NO_WAIT);
}
#endif

void matter_task_start(void);

int main(void) {
    LOG_INF("Sesame app starting up");
    leds_init();

    // Force timer inclusion for chip-gn
    struct k_timer dummy_timer;
    k_timer_init(&dummy_timer, NULL, NULL);
#ifdef CONFIG_SOC_88MW320
    init_watchdog();
#endif

    LOG_INF("Firmware version: %s", APP_VERSION_STRING);
    if (load_config() == 0) {
        LOG_INF("Config loaded successfully");
    } else {
        LOG_WRN("Failed to load config, using defaults");
    }

#ifdef CONFIG_BOOTLOADER_MCUBOOT
    check_ota_test_image();
    if (ota_status == OTA_STATUS_TESTING) {
        set_ota_led_pattern(LED_BLUE, LED_OFF, LED_BLUE, LED_OFF);
    } else
#endif
    {
        set_ota_led_pattern(LED_GREEN, LED_GREEN, LED_OFF, LED_OFF);
    }

    network_init();
    syslog_init();

#ifndef CONFIG_NET_L2_ETHERNET
    if (app_config.has_network_config &&
        strlen(app_config.network_config.ssid) > 0) {
        start_sta();
    } else {
        start_ap();
    }
#else
    LOG_INF("Ethernet enabled: AP/STA not started. Simulating L4_CONNECTED");
    k_event_post(&network_events, L4_UP_EVENT);
    k_event_post(&network_events, IPV4_UP_EVENT);
    k_event_post(&network_events, IPV6_UP_EVENT);
#endif

    http_server_start();

#if DT_NODE_EXISTS(DT_NODELABEL(sw_wifi))
    if (gpio_is_ready_dt(&wifi_button)) {
        gpio_pin_configure_dt(&wifi_button, GPIO_INPUT | GPIO_PULL_UP);
        gpio_pin_interrupt_configure_dt(&wifi_button, GPIO_INT_EDGE_TO_ACTIVE);
        gpio_init_callback(&wifi_button_cb_data, wifi_button_pressed,
                           BIT(wifi_button.pin));
        gpio_add_callback(wifi_button.port, &wifi_button_cb_data);
    }

#endif
    LOG_INF("System ready");

#ifdef CONFIG_CHIP
    matter_task_start();
#endif

    while (1) {
        ctrl_msg_t msg;
        int get_ret = k_msgq_get(&ctrl_queue, &msg, K_MSEC(1000));
        if (get_ret == 0) {
            switch (msg.type) {
                case CTRL_MSG_WIFI_BUTTON:
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
                case CTRL_MSG_DOOR_CONTROL: {
                    pic_cmd_t pcmd = PIC_CMD_UNKNOWN;
                    switch (msg.msg.door_control.command) {
                        case DOOR_CMD_OPEN:
                            pcmd = PIC_CMD_OPEN;
                            break;
                        case DOOR_CMD_CLOSE:
                            pcmd = PIC_CMD_CLOSE;
                            break;
                        case DOOR_CMD_STOP:
                            pcmd = PIC_CMD_STOP;
                            break;
                        default:
                            break;
                    }

                    if (pcmd != PIC_CMD_UNKNOWN) {
                        k_msgq_put(&pic_queue, &pcmd, K_NO_WAIT);
                    }
                    break;
                }
#ifdef CONFIG_BOOTLOADER_MCUBOOT
                case CTRL_MSG_OTA_UPGRADE:
                    ota_client_start(&msg.msg.ota_upgrade);
                    break;
                case CTRL_MSG_OTA_PROMOTE:
                    ota_promote_image();
                    break;
#endif
                case CTRL_MSG_DOOR_STATE_UPDATE:
                    publish_state(&msg.msg.door_state);
#ifdef CONFIG_CHIP
                    matter_update_door_state(&msg.msg.door_state);
#endif
                    break;
                default:
                    break;
            }
        }

#ifdef CONFIG_SOC_88MW320
        feed_watchdog();
#endif
    }
    return 0;
}
