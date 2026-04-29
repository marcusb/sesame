/*
 * Sesame - Custom firmware for Genie 1155 garage door opener
 * Copyright (C) 2024-2026  Marcus
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_logging.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_ND.h"
#include "NetworkInterface.h"
#include "queue.h"
#include "task.h"

// wmsdk
#include "debug_console.h"
#include "psm.h"

// Application
#include "config_manager.h"
#include "controller.h"
#include "dhcp.h"
#include "httpd.h"
#include "leds.h"
#include "logging.h"
#include "mqtt.h"
#include "network.h"
#include "ota.h"
#include "pic_uart.h"
#include "syslog.h"
#include "time_util.h"

#define mainLOGGING_MESSAGE_QUEUE_LENGTH (16)

// the global controller event queue, all tasks write to it
QueueHandle_t ctrl_queue;
psm_hnd_t psm_hnd;

QueueHandle_t ota_queue = NULL;
QueueHandle_t pic_queue = NULL;
QueueHandle_t nm_queue = NULL;

void log_console(const log_msg_t* log) {
    static const char levels[] = "-EWID";
    uint8_t level = log->level;
    if (level > LOG_LEVEL_LAST || level < 0) {
        level = LOG_NONE;
    }
    PRINTF("%lu %lu %c[%s] %s\r\n", log->msg_id, log->ticks, levels[level],
           log->task_name, log->msg);
}

void psm_init() {
#ifdef QEMU
    psm_module_init(NULL, &psm_hnd, NULL);
#else
    static flash_desc_t fl;
    int ret = part_get_desc_from_id(FC_COMP_PSM, &fl);
    if (ret != WM_SUCCESS) {
        LogError(("Unable to get flash desc from id"));
    }

    ret = psm_module_init(&fl, &psm_hnd, NULL);
    if (ret != 0) {
        LogError(("Failed to initialize psm module\r\n"));
    }
#endif
}

extern void board_init(void);
extern void configure_netif();
extern void create_board_tasks();
extern void reboot();
extern void notify_board_dhcp_configured();
extern void notify_board_ipv6_addr_change();
extern bool is_sta_iface(NetworkInterface_t* iface);
extern bool is_uap_iface(NetworkInterface_t* iface);

extern void WDT_Refresh(void* base);

void main_task(void* param) {
    board_init();

    register_log_backend(log_console);
    register_log_backend(log_syslog);
    init_logging(512, tskIDLE_PRIORITY, mainLOGGING_MESSAGE_QUEUE_LENGTH);

    check_ota_test_image();
    psm_init();
    load_config();
#ifndef QEMU
    start_rtc_save();
#endif

    configure_netif();

    ctrl_queue = xQueueCreate(8, sizeof(ctrl_msg_t));
    configASSERT(ctrl_queue);

    create_board_tasks();
    xTaskCreate(httpd_task, "HTTPd", 512, NULL, tskIDLE_PRIORITY, NULL);

    if (ota_status == OTA_STATUS_TESTING) {
        set_ota_led_pattern(LED_BLUE, LED_OFF, LED_BLUE, LED_OFF);
    } else {
        set_ota_led_pattern(LED_GREEN, LED_GREEN, LED_OFF, LED_OFF);
    }

    ctrl_msg_t ctrl_msg;
    for (;;) {
        WDT_Refresh(NULL);

        if (xQueueReceive(ctrl_queue, &ctrl_msg, 1000) == pdPASS) {
            switch (ctrl_msg.type) {
                case CTRL_MSG_OTA_UPGRADE: {
                    if (ota_queue) {
                        ota_msg_t ota_msg = {OTA_CMD_UPGRADE,
                                             .msg = {ctrl_msg.msg.ota_upgrade}};
                        xQueueSendToBack(ota_queue, &ota_msg, 0);
                    }
                    break;
                }

                case CTRL_MSG_OTA_PROMOTE: {
                    if (ota_queue) {
                        ota_msg_t ota_msg = {OTA_CMD_PROMOTE};
                        xQueueSendToBack(ota_queue, &ota_msg, 0);
                    }
                    break;
                }

                case CTRL_MSG_DOOR_CONTROL: {
                    if (pic_queue) {
                        pic_cmd_t cmd = PIC_CMD_UNKNOWN;
                        switch (ctrl_msg.msg.door_control.command) {
                            case DOOR_CMD_OPEN:
                                cmd = PIC_CMD_OPEN;
                                break;
                            case DOOR_CMD_CLOSE:
                                cmd = PIC_CMD_CLOSE;
                                break;
                            case DOOR_CMD_STOP:
                                cmd = PIC_CMD_STOP;
                                break;
                            default:
                                break;
                        }
                        if (cmd != PIC_CMD_UNKNOWN) {
                            xQueueSendToBack(pic_queue, &cmd, 0);
                        }
                    }
                    break;
                }

                case CTRL_MSG_DOOR_STATE_UPDATE:
                    publish_state(&ctrl_msg.msg.door_state);
                    break;

                case CTRL_MSG_WIFI_BUTTON: {
                    if (nm_queue) {
                        nm_msg_t cmd = {NM_CMD_AP_MODE};
                        LogDebug(("WiFi button pressed"));
                        xQueueSendToBack(nm_queue, &cmd, 0);
                    }
                    break;
                }

                case CTRL_MSG_OTA_BUTTON:
                    LogDebug(("OTA button pressed"));
                    reboot();
                    break;

                case CTRL_MSG_WIFI_CONFIG: {
                    app_config.network_config = ctrl_msg.msg.network_cfg;
                    app_config.has_network_config = true;
                    save_config();
                    reboot();
                    break;
                }

                case CTRL_MSG_MQTT_CONFIG: {
                    app_config.mqtt_config = ctrl_msg.msg.mqtt_cfg;
                    app_config.has_mqtt_config = true;
                    save_config();
                    reboot();
                    break;
                }

                case CTRL_MSG_LOGGING_CONFIG: {
                    app_config.logging_config = ctrl_msg.msg.logging_cfg;
                    app_config.has_logging_config = true;
                    save_config();
                    reboot();
                    break;
                }

                default:
                    LogError(("unknown message type %d", ctrl_msg.type));
            }
        }
    }
}

void print_ip_config(NetworkEndPoint_t* endpoint) {
    if (endpoint != NULL) {
        char ip_addr_s[40];
        char netmask_s[16];
        char gateway_s[40];
        char dns_server_s[40];
#if ipconfigUSE_IPv6
        if (endpoint->bits.bIPv6) {
            FreeRTOS_inet_ntop6(&endpoint->ipv6_settings.xIPAddress, ip_addr_s,
                                sizeof(ip_addr_s));
            FreeRTOS_inet_ntop6(&endpoint->ipv6_settings.xGatewayAddress,
                                gateway_s, sizeof(gateway_s));
            FreeRTOS_inet_ntop6(&endpoint->ipv6_settings.xDNSServerAddresses[0],
                                dns_server_s, sizeof(dns_server_s));
            LogInfo(("%s: IPv6 %s/%d, gw %s, dns %s",
                     endpoint->pxNetworkInterface->pcName, ip_addr_s,
                     endpoint->ipv6_settings.uxPrefixLength, gateway_s,
                     dns_server_s));
        } else
#endif
        {
            FreeRTOS_inet_ntop4(&endpoint->ipv4_settings.ulIPAddress, ip_addr_s,
                                sizeof(ip_addr_s));
            FreeRTOS_inet_ntop4(&endpoint->ipv4_settings.ulNetMask, netmask_s,
                                sizeof(netmask_s));
            FreeRTOS_inet_ntop4(&endpoint->ipv4_settings.ulGatewayAddress,
                                gateway_s, sizeof(gateway_s));
            FreeRTOS_inet_ntop4(
                &endpoint->ipv4_settings.ulDNSServerAddresses[0], dns_server_s,
                sizeof(dns_server_s));
            LogInfo(("%s: IPv4 %s/%s, gw %s, dns %s",
                     endpoint->pxNetworkInterface->pcName, ip_addr_s, netmask_s,
                     gateway_s, dns_server_s));
        }
    }
}

void vApplicationMallocFailedHook() {
    PRINTF("ERROR: Malloc failed to allocate memory\r\n");
    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char* task_name) {
    PRINTF("ERROR: stack overflow in task %s\r\n", task_name);
    portDISABLE_INTERRUPTS();
    for (;;) {
    }
}

BaseType_t xApplicationDNSQueryHook_Multi(struct xNetworkEndPoint* pxEndPoint,
                                          const char* pcName) {
    (void)pxEndPoint;
    (void)pcName;
    return pdFALSE;
}

#if SESAME_ENABLE_MATTER
extern void matter_init(void);
#endif

void vApplicationIPNetworkEventHook_Multi(eIPCallbackEvent_t event,
                                          NetworkEndPoint_t* endpoint) {
    static bool tasks_created = false;

    if (event == eNetworkUp) {
        LogInfo(("%s: interface up", endpoint->pxNetworkInterface->pcName));
        print_ip_config(endpoint);

        if (is_sta_iface(endpoint->pxNetworkInterface)) {
#if ipconfigUSE_IPv6
            if (endpoint->bits.bIPv6) {
                notify_board_ipv6_addr_change();
            } else
#endif
            {
                notify_board_dhcp_configured();
            }
            if (!tasks_created) {
                tasks_created = true;
                configure_logging(&app_config.logging_config.syslog_config);
#ifndef QEMU
                xTaskCreate(mqtt_task, "MQTT", 512, &app_config.mqtt_config,
                            tskIDLE_PRIORITY, NULL);
#endif
#if SESAME_ENABLE_MATTER
                matter_init();
#endif
            }
        } else if (is_uap_iface(endpoint->pxNetworkInterface)) {
            static dhcp_task_params_t dhcp_params;
            dhcp_params.endpoint = endpoint;
            xTaskCreate(dhcpd_task, "DHCPd", 512, &dhcp_params,
                        tskIDLE_PRIORITY, NULL);
        } else if (tasks_created == false) {
            // QEMU or other generic interface
            tasks_created = true;
            configure_logging(&app_config.logging_config.syslog_config);
            xTaskCreate(mqtt_task, "MQTT", 512, &app_config.mqtt_config,
                        tskIDLE_PRIORITY, NULL);
#if SESAME_ENABLE_MATTER
            matter_init();
#endif
        }
    }
}
