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
#include "iot_wifi.h"
#include "mflash_drv.h"
#include "partition.h"
#include "wlan.h"

// Application
#include "backoff_algorithm.h"
#include "config_manager.h"
#include "network.h"
#include "string_util.h"
#include "time_util.h"

#define WIFI_CONNECTION_BACKOFF_BASE_MS (10 * 1000)
#define WIFI_CONNECTION_RETRIES (2)

typedef enum {
    STA_IDLE,
    STA_CONNECTING,
    STA_CONNECTED,
    STA_CONNECT_FAILED
} connection_state_t;

static connection_state_t network_state = STA_IDLE;
static struct wlan_network client_network;
static QueueHandle_t nm_queue;
static BackoffAlgorithmContext_t connect_retry_params;
static struct wlan_network ap_network;
static TickType_t mark;
static TickType_t connect_interval;
static char hostname[33] = "sesame";

const char *pcApplicationHostnameHook() { return hostname; }

static int get_network_config(WIFINetworkParams_t *params) {
    char *p = stpncpy((char *)params->ucSSID, app_config.network_config.ssid,
                      sizeof(params->ucSSID));
    int n = p - (char *)params->ucSSID;
    params->ucSSIDLength = n;
    if (n == 0) {
        return -1;
    }

    p = stpncpy((char *)params->xPassword.xWPA.cPassphrase,
                app_config.network_config.password,
                sizeof(params->xPassword.xWPA.cPassphrase));
    n = p - (char *)params->xPassword.xWPA.cPassphrase;
    params->xPassword.xWPA.ucLength = n;
    params->xSecurity = eWiFiSecurityWPA2;
    if (n == 0) {
        return -1;
    }

    n = strtcpy(hostname, app_config.network_config.hostname, sizeof(hostname));
    if (n <= 0) {
        strtcpy(hostname, "sesame", sizeof(hostname));
    }
    return 0;
}

static void connect_attempt_failed() {
    uint16_t backoff;
    BackoffAlgorithmStatus_t retry_status = BackoffAlgorithm_GetNextBackoff(
        &connect_retry_params, rand(), &backoff);
    if (retry_status == BackoffAlgorithmRetriesExhausted) {
        network_state = STA_CONNECT_FAILED;
    } else {
        LogInfo(("connection attempt %d of %d failed, next in %d ms",
                 connect_retry_params.attemptsDone,
                 connect_retry_params.maxRetryAttempts + 1, backoff));
        mark = xTaskGetTickCount();
        connect_interval = pdMS_TO_TICKS(backoff);
    }
}

static int init_sta_network(void) {
    connect_interval = portMAX_DELAY;
    WIFINetworkParams_t wifi_params = {0};
    int res = get_network_config(&wifi_params);
    if (res < 0) {
        goto fail;
    }

    memset(&client_network, 0, sizeof(client_network));
    strcpy(client_network.name, "client");
    if (wifi_params.ucSSIDLength > IEEEtypes_SSID_SIZE) {
        LogDebug(("SSID too long"));
        goto fail;
    }
    memcpy(client_network.ssid, wifi_params.ucSSID, wifi_params.ucSSIDLength);

    if (wifi_params.xPassword.xWPA.ucLength > WLAN_PSK_MAX_LENGTH - 1) {
        // one extra char for null
        LogDebug(("WPA password too long"));
        goto fail;
    }
    memcpy(client_network.security.psk, wifi_params.xPassword.xWPA.cPassphrase,
           wifi_params.xPassword.xWPA.ucLength);
    client_network.security.psk_len = wifi_params.xPassword.xWPA.ucLength;

    switch (wifi_params.xSecurity) {
        case eWiFiSecurityOpen:
            client_network.security.type = WLAN_SECURITY_NONE;
            break;
        case eWiFiSecurityWEP:
            client_network.security.type = WLAN_SECURITY_WEP_OPEN;
            break;
        case eWiFiSecurityWPA:
            client_network.security.type = WLAN_SECURITY_WPA;
            break;
        case eWiFiSecurityWPA2:
            client_network.security.type = WLAN_SECURITY_WPA2;
            break;
        default:
            LogDebug(("unsupported wifi sec type"));
            goto fail;
    }
    client_network.type = WLAN_BSS_TYPE_STA;
    client_network.role = WLAN_BSS_ROLE_STA;
    client_network.channel = wifi_params.ucChannel;
    client_network.ip.ipv4.addr_type = ADDR_TYPE_DHCP;

    wlan_remove_network(client_network.name);
    res = wlan_add_network(&client_network);
    if (res != WM_SUCCESS) {
        LogError(("wlan_add_network failed %d", res));
        goto fail;
    }
    return 0;

fail:
    network_state = STA_CONNECT_FAILED;
    return -1;
}

static int connect_sta(void) {
    LogInfo(("connecting to wlan %s", client_network.ssid));
    int res = wlan_connect(client_network.name);
    if (res != WM_SUCCESS) {
        LogError(("wlan_connect failed %d", res));
        goto fail;
    }
    return 0;

fail:
    network_state = STA_CONNECT_FAILED;
    return -1;
}

static void reconnect_sta() {
    if (network_state == STA_CONNECTING) {
        connect_sta();
    }
}

static void start_sta_connection(void) {
    BackoffAlgorithm_InitializeParams(
        &connect_retry_params, WIFI_CONNECTION_BACKOFF_BASE_MS,
        WIFI_CONNECTION_BACKOFF_BASE_MS * 2, WIFI_CONNECTION_RETRIES);
    network_state = STA_CONNECTING;
    connect_sta();
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

static int wlan_event_callback(enum wlan_event_reason event, void *data) {
    switch (event) {
        case WLAN_REASON_INITIALIZED: {
            wifi_fw_version_ext_t ver;
            wifi_get_device_firmware_version_ext(&ver);
            LogDebug(("wifi initialized, wlan fw v%s", ver.version_str));
            init_ap_network();
            if (init_sta_network() == 0) {
                start_sta_connection();
            }
            break;
        }
        case WLAN_REASON_SUCCESS:
            LogDebug(("wifi connected"));
            break;
        case WLAN_REASON_CONNECT_FAILED:
            LogDebug(("wifi connect failed (invalid arg)"));
            connect_attempt_failed();
            break;
        case WLAN_REASON_NETWORK_NOT_FOUND:
            LogDebug(("wifi connect failed (network not found)"));
            connect_attempt_failed();
            break;
        case WLAN_REASON_NETWORK_AUTH_FAILED:
            LogDebug(("wifi connect failed (auth failed)"));
            connect_attempt_failed();
            break;
        case WLAN_REASON_ADDRESS_FAILED:
            LogDebug(("wifi disconnected (failed to obtain address)"));
            connect_attempt_failed();
            break;
        case WLAN_REASON_LINK_LOST:
            LogDebug(("wifi disconnected (link lost)"));
            break;
        case WLAN_REASON_USER_DISCONNECT:
            LogDebug(("wifi disconnected by user request"));
            reconnect_sta();
            break;
        case WLAN_REASON_UAP_SUCCESS:
            LogDebug(("wifi AP started"));
            break;
        case WLAN_REASON_UAP_STOPPED:
            LogDebug(("wifi AP stopped"));
            break;
        default:
            LogDebug(("wifi event %d", event));
    }

    return 0;
}

void start_ap() {
    LogInfo(("Starting access point"));
    int res = wlan_start_network("ap");
    if (res != WM_SUCCESS) {
        LogInfo(("failed to start AP: %d", res));
        return;
    }
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
        if (xQueueReceive(nm_queue, &cmd, pdMS_TO_TICKS(200)) == pdPASS) {
            switch (cmd.type) {
                case NM_CMD_AP_MODE:
                    start_ap();
                    break;

                default:
                    LogError(("unknown nm_cmd_t %d", cmd));
            }
        }
        if (network_state == STA_CONNECTING &&
            xTaskGetTickCount() - mark > connect_interval) {
            connect_sta();
        } else if (network_state == STA_CONNECT_FAILED) {
            network_state = STA_IDLE;
            start_ap();
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
