#include <limits.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "queue.h"

// wmsdk
#include "iot_wifi.h"
#include "wlan.h"
#include "wm_wlan.h"

// Application
#include "app_logging.h"
#include "backoff_algorithm.h"
#include "config_manager.h"
#include "network.h"
#include "string_util.h"
#include "time_util.h"

#define WIFI_CONNECTION_BACKOFF_BASE_MS (1000)
#define WIFI_CONNECTION_RETRIES (2)

typedef enum {
    STA_IDLE,
    STA_CONNECTING,
    STA_CONNECTED,
    STA_CONNECT_FAILED
} connection_state_t;

static connection_state_t network_state = STA_IDLE;
static QueueHandle_t nm_queue;
static BackoffAlgorithmContext_t connect_retry_params;
static struct wlan_network ap_network;
static TickType_t mark;
static TickType_t connect_interval;
static char hostname[33] = "sesame";

const char *pcApplicationHostnameHook() { return hostname; }

/**
 * @brief Function to set a memory block to zero.
 * The function sets memory to zero using a volatile pointer so that
 * compiler wont optimize out the function if the buffer to be set to zero
 * is not used further.
 *
 * @param pBuf Pointer to buffer to be set to zero
 * @param size Length of the buffer to be set zero
 */
static void mem_clear(void *pBuf, size_t size) {
    volatile uint8_t *pMem = pBuf;
    uint32_t i;
    for (i = 0U; i < size; i++) {
        pMem[i] = 0U;
    }
}

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

static int connect_sta(void) {
    connect_interval = portMAX_DELAY;
    WIFINetworkParams_t wifi_params = {0};
    int res = get_network_config(&wifi_params);
    if (res < 0) {
        goto fail;
    }

    struct wlan_network nw;
    memset(&nw, 0, sizeof(nw));
    strcpy(nw.name, "client");
    if (wifi_params.ucSSIDLength > IEEEtypes_SSID_SIZE) {
        LogDebug(("SSID too long"));
        goto fail;
    }
    memcpy(nw.ssid, wifi_params.ucSSID, wifi_params.ucSSIDLength);

    if (wifi_params.xPassword.xWPA.ucLength > WLAN_PSK_MAX_LENGTH - 1) {
        // one extra char for null
        LogDebug(("WPA password too long"));
        goto fail;
    }
    memcpy(nw.security.psk, wifi_params.xPassword.xWPA.cPassphrase,
           wifi_params.xPassword.xWPA.ucLength);
    nw.security.psk_len = wifi_params.xPassword.xWPA.ucLength;

    switch (wifi_params.xSecurity) {
        case eWiFiSecurityOpen:
            nw.security.type = WLAN_SECURITY_NONE;
            break;
        case eWiFiSecurityWEP:
            nw.security.type = WLAN_SECURITY_WEP_OPEN;
            break;
        case eWiFiSecurityWPA:
            nw.security.type = WLAN_SECURITY_WPA;
            break;
        case eWiFiSecurityWPA2:
            nw.security.type = WLAN_SECURITY_WPA2;
            break;
        default:
            LogDebug(("unsupported wifi sec type"));
            goto fail;
    }
    nw.type = WLAN_BSS_TYPE_STA;
    nw.role = WLAN_BSS_ROLE_STA;
    nw.channel = wifi_params.ucChannel;
    nw.ip.ipv4.addr_type = ADDR_TYPE_DHCP;

    wlan_remove_network(nw.name);
    res = wlan_add_network(&nw);
    if (res != WM_SUCCESS) {
        LogError(("wlan_add_network failed %d", res));
        goto fail;
    }

    LogInfo(("connecting to wlan %s", nw.ssid));
    res = wlan_connect(nw.name);
    if (res != WM_SUCCESS) {
        LogError(("wlan_connect failed %d", res));
        goto fail;
    }
    /*
     * Use a private function to reset the memory block instead of memset,
     * so that compiler wont optimize away the function call.
     */
    mem_clear(&wifi_params, sizeof(WIFINetworkParams_t));
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

static void start_ap_connection(void) {
    BackoffAlgorithm_InitializeParams(
        &connect_retry_params, WIFI_CONNECTION_BACKOFF_BASE_MS,
        WIFI_CONNECTION_BACKOFF_BASE_MS * 2, WIFI_CONNECTION_RETRIES);
    network_state = STA_CONNECTING;
    // wlan_disconnect will trigger callback with WLAN_REASON_USER_DISCONNECT,
    // where we will start the connection attempt
    if (wlan_disconnect() != WM_SUCCESS) {
        // we were already disconnected
        reconnect_sta();
    }
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
        case WLAN_REASON_INITIALIZED:
            LogDebug(("wifi initialized"));
            init_ap_network();
            start_ap_connection();
            break;
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

void network_manager_task(void *params) {
    nm_queue = (QueueHandle_t)params;

    int res = wm_wlan_init();
    if (res != WM_SUCCESS) {
        LogError(("wifi init failed %d", res));
        goto err;
    }

    res = wlan_start(wlan_event_callback);
    if (res != WM_SUCCESS) {
        LogError(("wlan_start failed %d", res));
        goto err;
    }

    // wlan_set_country(COUNTRY_US);
    // WIFIScanResult_t scan[6];
    // int res = WIFI_Scan(scan, 6);
    // if (res) {
    //     LogError(("scan failed"));
    //     goto err;
    // }
    // for (int i = 0; i < 6; i++) {
    //     LogInfo(("%.*s", scan[i].ucSSIDLength, scan[i].ucSSID));
    // }

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
            wmprintf("\r\nfree heap space     : %u\r\n",
                     stats.xAvailableHeapSpaceInBytes);
            wmprintf("largest free block  : %u\r\n",
                     stats.xSizeOfLargestFreeBlockInBytes);
            wmprintf("smallest free block : %u\r\n",
                     stats.xSizeOfSmallestFreeBlockInBytes);
            wmprintf("free block count    : %u\r\n", stats.xNumberOfFreeBlocks);
            wmprintf("min free heap ever  : %u\r\n",
                     stats.xMinimumEverFreeBytesRemaining);
            wmprintf("successful malloc   : %u\r\n",
                     stats.xNumberOfSuccessfulAllocations);
            wmprintf("successful free     : %u\r\n\r\n",
                     stats.xNumberOfSuccessfulFrees);
        }
    }

err:
    vTaskDelete(NULL);
}
