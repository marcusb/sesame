#include <stdbool.h>
#include <string.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "queue.h"

// wmsdk
#include "psm-v2.h"
#include "wlan.h"

// Application
#include "app_logging.h"
#include "dhcp.h"
#include "iot_wifi.h"
#include "network.h"

typedef enum { NW_DISCONNECTED, NW_MODE_STA, NW_MODE_AP } NetworkState_t;

static NetworkState_t network_state;
static QueueHandle_t nm_queue;
static psm_hnd_t psm_hnd;
static const char psm_key_ssid[] = "wlan.ssid";
static const char psm_key_wlan_passwd[] = "wlan.passwd";

void print_ip_config(NetworkEndPoint_t *endpoint) {
    if (endpoint != NULL) {
        uint32_t ip_addr;
        uint32_t netmask;
        uint32_t gateway;
        uint32_t dns_server;
        FreeRTOS_GetEndPointConfiguration(&ip_addr, &netmask, &gateway,
                                          &dns_server, endpoint);

        char ip_addr_s[16];
        char netmask_s[16];
        char gateway_s[16];
        char dns_server_s[16];
        FreeRTOS_inet_ntop4(&ip_addr, ip_addr_s, sizeof(ip_addr_s));
        FreeRTOS_inet_ntop4(&netmask, netmask_s, sizeof(netmask_s));
        FreeRTOS_inet_ntop4(&gateway, gateway_s, sizeof(gateway_s));
        FreeRTOS_inet_ntop4(&dns_server, dns_server_s, sizeof(dns_server_s));
        LogInfo(("IPv4 %s/%s, gw %s, dns %s", ip_addr_s, netmask_s, gateway_s,
                 dns_server_s));
    }
}

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

#define WIFI_CONNECTION_RETRY_INTERVAL_MS (1000)
#define WIFI_CONNECTION_RETRIES (5)

static int get_network_config(WIFINetworkParams_t *params) {
    int ret = psm_get_variable(psm_hnd, psm_key_ssid, params->ucSSID,
                               sizeof(params->ucSSID));
    if (ret < 0) {
        LogDebug(("psm read ssid failed %d", ret));
        return ret;
    }
    params->ucSSIDLength = ret;

    ret = psm_get_variable(psm_hnd, psm_key_wlan_passwd,
                           params->xPassword.xWPA.cPassphrase,
                           sizeof(params->xPassword.xWPA.cPassphrase));
    if (ret < 0) {
        LogDebug(("psm read passwd failed %d", ret));
        return ret;
    }
    params->xPassword.xWPA.ucLength = ret;
    params->xSecurity = eWiFiSecurityWPA2;
    return 0;
}

static int set_network_config(WIFINetworkParams_t params) {
    int ret = psm_set_variable(psm_hnd, psm_key_ssid, params.ucSSID,
                               params.ucSSIDLength);
    if (ret < 0) {
        LogError(("psm write ssid failed %d", ret));
        return ret;
    }
    ret = psm_set_variable(psm_hnd, psm_key_wlan_passwd,
                           params.xPassword.xWPA.cPassphrase,
                           params.xPassword.xWPA.ucLength);
    if (ret < 0) {
        LogError(("psm write passwd failed %d", ret));
        return ret;
    }
    return 0;
}

static int connect_sta(void) {
    WIFINetworkParams_t wifi_params = {0};
    int res = get_network_config(&wifi_params);
    if (res < 0) {
        return res;
    }

    uint32_t numRetries = WIFI_CONNECTION_RETRIES;
    uint32_t delayMilliseconds = WIFI_CONNECTION_RETRY_INTERVAL_MS;
    do {
        if (WIFI_ConnectAP(&(wifi_params)) == eWiFiSuccess) {
            break;
        }
        if (numRetries > 0) {
            vTaskDelay(pdMS_TO_TICKS(delayMilliseconds));
            delayMilliseconds = delayMilliseconds * 2;
        } else {
            res = -1;
        }
    } while (numRetries-- > 0);

    /*
     * Use a private function to reset the memory block instead of memset,
     * so that compiler wont optimize away the function call.
     */
    mem_clear(&wifi_params, sizeof(WIFINetworkParams_t));
    return 0;
}

static int wifi_connect(void) {
    int ret = -1;
    if (network_state == NW_DISCONNECTED) {
        ret = connect_sta();
        if (ret == 0) {
            network_state = NW_MODE_STA;
        }
    }
    return ret;
}

static struct wlan_network ap_network;
static void init_ap_network() {
    wlan_initialize_uap_network(&ap_network);
    strncpy(ap_network.name, "ap", sizeof(ap_network.name));
    strncpy(ap_network.ssid, "sesame", sizeof(ap_network.ssid));
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
    int res = wlan_start_network("ap");
    if (res != WM_SUCCESS) {
        LogInfo(("failed to start AP: %d", res));
        return;
    }
    network_state = NW_MODE_AP;

    dhcp_task_params_t params = {ap_network.ip.ipv4.address,
                                 ap_network.ip.ipv4.netmask};
    xTaskCreate(dhcpd_task, "dhcpd", 512, &params, tskIDLE_PRIORITY, NULL);
}

static void try_connect_wifi() {
    int res = wifi_connect();
    if (res < 0) {
        LogInfo(("no wlan connection, starting AP"));
        start_ap();
    }
}

static void run() {
    if (WIFI_On() != eWiFiSuccess) {
        LogError(("wifi init failed"));
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

    init_ap_network();
    try_connect_wifi();

    for (;;) {
        vTaskDelay(1000);
        nm_msg_t cmd;
        if (xQueueReceive(nm_queue, &cmd, portMAX_DELAY) == pdPASS) {
            switch (cmd.type) {
                case NM_CMD_AP_MODE:
                    start_ap();
                    break;

                case NM_CMD_WIFI_CONFIG:
                    set_network_config(cmd.msg.wifi_cfg.network_params);
                    try_connect_wifi();
                    break;

                default:
                    LogError(("unknown nm_cmd_t %d", cmd));
            }
        }
    }

err:
    vTaskDelete(NULL);
}

void network_manager_task(void *params) {
    nm_task_params_t *nm_params = (nm_task_params_t *)params;
    nm_queue = nm_params->nm_queue;
    psm_hnd = nm_params->psm_hnd;

    run();
}
