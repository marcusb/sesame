#include <limits.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "app_logging.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "queue.h"

// wmsdk
#include "leds.h"
#include "mflash_drv.h"
#include "partition.h"
#include "wlan.h"

// Application
#include "config_manager.h"
#include "network.h"
#include "string_util.h"
#include "time_util.h"

#define RECONNECT_WAIT_TICKS pdMS_TO_TICKS(10000)

typedef enum { STA_IDLE, STA_CONNECT_FAILED } connection_state_t;

static bool sta_initialized;
static TickType_t last_conn_attempt;
static connection_state_t network_state = STA_IDLE;
static struct wlan_network client_network;
static QueueHandle_t nm_queue;
static struct wlan_network ap_network;
static char hostname[33] = "sesame";

const char *pcApplicationHostnameHook() { return hostname; }

static void connect_attempt_failed() {
    set_wifi_led_pattern(LED_OFF, LED_OFF, LED_OFF, LED_OFF);
    last_conn_attempt = xTaskGetTickCount();
    network_state = STA_CONNECT_FAILED;
}

static int init_sta_network(void) {
    memset(&client_network, 0, sizeof(client_network));
    strcpy(client_network.name, "client");
    stpncpy(client_network.ssid, app_config.network_config.ssid,
            sizeof(client_network.ssid));
    char *p =
        stpncpy(client_network.security.psk, app_config.network_config.password,
                sizeof(client_network.security.psk));
    int pass_len = p - client_network.security.psk;
    client_network.security.psk_len = pass_len;
    client_network.security.type = WLAN_SECURITY_WPA2;
    client_network.type = WLAN_BSS_TYPE_STA;
    client_network.role = WLAN_BSS_ROLE_STA;
    client_network.ip.ipv4.addr_type = ADDR_TYPE_DHCP;

    if (strtcpy(hostname, app_config.network_config.hostname,
                sizeof(hostname)) <= 0) {
        strtcpy(hostname, "sesame", sizeof(hostname));
    }

    wlan_remove_network(client_network.name);
    int res = wlan_add_network(&client_network);
    if (res == WM_SUCCESS) {
        return 0;
    } else {
        LogError(("wlan_add_network failed %d", res));
        return -1;
    }
}

static int connect_sta(void) {
    LogInfo(("connecting to wlan %s", client_network.ssid));
    int res = wlan_connect(client_network.name);
    if (res != WM_SUCCESS) {
        LogError(("wlan_connect failed %d", res));
        goto fail;
    }
    set_wifi_led_pattern(LED_GREEN, LED_OFF, LED_GREEN, LED_OFF);
    return 0;

fail:
    connect_attempt_failed();
    return -1;
}

static void init_ap_network() {
    wlan_initialize_uap_network(&ap_network);
    strcpy(ap_network.name, "ap");
    strcpy(ap_network.ssid, "sesame");
    FreeRTOS_inet_pton(FREERTOS_AF_INET, "192.168.4.1",
                       &ap_network.ip.ipv4.address);
    FreeRTOS_inet_pton(FREERTOS_AF_INET, "255.255.255.0",
                       &ap_network.ip.ipv4.netmask);
    FreeRTOS_inet_pton(FREERTOS_AF_INET, "192.168.4.1", &ap_network.ip.ipv4.gw);
    ap_network.security.type = WLAN_SECURITY_NONE;
    int ret = wlan_add_network(&ap_network);
    if (ret != WM_SUCCESS) {
        LogWarn(("network_mgr: failed to add network %d", ret));
    }
}

void start_ap() {
    static bool ap_started = false;
    if (!ap_started) {
        LogInfo(("Starting access point"));
        int res = wlan_start_network("ap");
        if (res == WM_SUCCESS) {
            ap_started = true;
        } else {
            LogInfo(("failed to start AP: %d", res));
        }
    }
}

static int wlan_event_callback(enum wlan_event_reason event, void *data) {
    const char *msg;
    switch (event) {
        case WLAN_REASON_INITIALIZED: {
            init_ap_network();
            msg = "wlan initialized";
            if (init_sta_network() == 0) {
                sta_initialized = true;
                connect_sta();
            } else {
                start_ap();
            }
            break;
        }
        case WLAN_REASON_SUCCESS:
            msg = "wifi connected";
            set_wifi_led_pattern(LED_GREEN, LED_GREEN, LED_GREEN, LED_GREEN);
            break;
        case WLAN_REASON_CONNECT_FAILED:
            msg = "wifi connect failed (invalid arg)";
            connect_attempt_failed();
            break;
        case WLAN_REASON_NETWORK_NOT_FOUND:
            msg = "wifi connect failed (network not found)";
            connect_attempt_failed();
            break;
        case WLAN_REASON_NETWORK_AUTH_FAILED:
            msg = "wifi connect failed (auth failed)";
            connect_attempt_failed();
            break;
        case WLAN_REASON_ADDRESS_FAILED:
            msg = "wifi disconnected (failed to obtain address)";
            connect_attempt_failed();
            break;
        case WLAN_REASON_LINK_LOST:
            msg = "wifi disconnected (link lost)";
            connect_attempt_failed();
            break;
        case WLAN_REASON_USER_DISCONNECT:
            msg = "wifi disconnected by user request";
            connect_attempt_failed();
            break;
        case WLAN_REASON_UAP_SUCCESS:
            set_wifi_led_pattern(LED_BLUE, LED_OFF, LED_BLUE, LED_OFF);
            msg = "wifi AP started";
            break;
        case WLAN_REASON_UAP_STOPPED:
            msg = "wifi AP stopped";
            break;
        case WLAN_REASON_PS_ENTER:
            msg = "PS_ENTER";
            break;
        case WLAN_REASON_PS_EXIT:
            msg = "PS EXIT";
            break;
        default:
            msg = "other";
    }
    LogDebug(("wlan event: %s (%d)", msg, event));

    return 0;
}

static int init_wifi_driver() {
    short history = 0;
    struct partition_entry *p1 =
        part_get_layout_by_id(FC_COMP_WLAN_FW, &history);
    struct partition_entry *p2 =
        part_get_layout_by_id(FC_COMP_WLAN_FW, &history);
    struct partition_entry *p;
    if (p1 && p2) {
        p = part_get_active_partition(p1, p2);
    } else if (!p1 && p2) {
        p = p2;
    } else if (!p2 && p1) {
        p = p1;
    } else {
        LogError(("Wi-Fi firmware missing"));
        return -1;
    }

    flash_desc_t fl;
    part_to_flash_desc(p, &fl);
    uint32_t *wififw = (uint32_t *)mflash_drv_phys2log(fl.fl_start, fl.fl_size);
    configASSERT(wififw != NULL);
    /* First word in WIFI firmware is magic number. */
    configASSERT(*wififw ==
                 (('W' << 0) | ('L' << 8) | ('F' << 16) | ('W' << 24)));

    /* Initialize WIFI Driver */
    /* Second word in WIFI firmware is WIFI firmware length in bytes. */
    /* Real WIFI binary starts from 3rd word. */
    return wlan_init((const uint8_t *)(wififw + 2U), *(wififw + 1U));
}

void network_manager_task(void *params) {
    nm_queue = (QueueHandle_t)params;

    int res = init_wifi_driver();
    if (res != WM_SUCCESS) {
        LogError(("wifi init failed %d", res));
        goto err;
    }

    res = wlan_start(wlan_event_callback);
    if (res != WM_SUCCESS) {
        LogError(("wlan_start failed %d", res));
        goto err;
    }
    wlan_set_country(COUNTRY_US);

    int n = 0;
    for (;;) {
        nm_msg_t cmd;
        if (xQueueReceive(nm_queue, &cmd, pdMS_TO_TICKS(500)) == pdPASS) {
            switch (cmd.type) {
                case NM_CMD_AP_MODE:
                    start_ap();
                    break;

                default:
                    LogError(("unknown nm_cmd_t %d", cmd));
            }
        }
        if (network_state == STA_CONNECT_FAILED) {
            start_ap();
            if (xTaskGetTickCount() - last_conn_attempt >
                RECONNECT_WAIT_TICKS) {
                LogDebug(("reconnecting to wifi"));
                connect_sta();
                network_state = STA_IDLE;
            }
        }
        if (n++ % 100 == 0) {
            static HeapStats_t stats;
            vPortGetHeapStats(&stats);
            PRINTF("\r\nfree heap space     : %u\r\n",
                   stats.xAvailableHeapSpaceInBytes);
            PRINTF("largest free block  : %u\r\n",
                   stats.xSizeOfLargestFreeBlockInBytes);
            PRINTF("smallest free block : %u\r\n",
                   stats.xSizeOfSmallestFreeBlockInBytes);
            PRINTF("free block count    : %u\r\n", stats.xNumberOfFreeBlocks);
            PRINTF("min free heap ever  : %u\r\n",
                   stats.xMinimumEverFreeBytesRemaining);
            PRINTF("successful malloc   : %u\r\n",
                   stats.xNumberOfSuccessfulAllocations);
            PRINTF("successful free     : %u\r\n\r\n",
                   stats.xNumberOfSuccessfulFrees);
        }
    }

err:
    vTaskDelete(NULL);
}
