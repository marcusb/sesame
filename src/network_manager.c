#include <stdbool.h>
#include <string.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "queue.h"

// wmsdk
#include "wlan.h"

// Application
#include "app_logging.h"
#include "dhcp.h"
#include "iot_wifi.h"
#include "network.h"

typedef enum NetworkState {
    NetworkStateDisabled = 0,
    NetworkStateEnabled = 1,
    NetworkStateConnected = 2,
} NetworkState_t;

NetworkState_t network_state;

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

#define _NM_WIFI_CONNECTION_RETRY_INTERVAL_MS (1000)
#define _NM_WIFI_CONNECTION_RETRIES (5)

static bool connect_ap(void) {
    bool status = true;
    WIFINetworkParams_t xConnectParams;
    size_t ssid_len, xPasswordLength;
    const char *ssid = "***REMOVED***";
    const char *password = "***REMOVED***";
    WIFISecurity_t xSecurity = eWiFiSecurityWPA2;
    uint32_t numRetries = _NM_WIFI_CONNECTION_RETRIES;
    uint32_t delayMilliseconds = _NM_WIFI_CONNECTION_RETRY_INTERVAL_MS;

    if (ssid != NULL) {
        ssid_len = strlen(ssid);
        if (ssid_len > 0 && ssid_len < wificonfigMAX_SSID_LEN) {
            xConnectParams.ucSSIDLength = ssid_len;
            memcpy(xConnectParams.ucSSID, ssid, ssid_len);
        } else {
            status = false;
        }
    } else {
        status = false;
    }

    xConnectParams.xSecurity = xSecurity;
    switch (xSecurity) {
        case eWiFiSecurityWPA:
        case eWiFiSecurityWPA2:

            if (password != NULL) {
                xPasswordLength = strlen(password);

                if (xPasswordLength > 0 &&
                    xPasswordLength < wificonfigMAX_PASSPHRASE_LEN) {
                    xConnectParams.xPassword.xWPA.ucLength = xPasswordLength;
                    memcpy(xConnectParams.xPassword.xWPA.cPassphrase, password,
                           xPasswordLength);
                } else {
                    status = false;
                }
            } else {
                status = false;
            }

            break;

        case eWiFiSecurityOpen:
            /* Nothing to do. */
            break;

        case eWiFiSecurityWPA3:
        case eWiFiSecurityWPA2_ent:
        case eWiFiSecurityWEP:
        default:
            LogError(("The configured WiFi security option is not supported."));
            status = false;
            break;
    }

    if (status == true) {
        /* Try to connect to wifi access point with retry and exponential
         * delay
         */
        do {
            if (WIFI_ConnectAP(&(xConnectParams)) == eWiFiSuccess) {
                break;
            } else {
                if (numRetries > 0) {
                    vTaskDelay(pdMS_TO_TICKS(delayMilliseconds));
                    delayMilliseconds = delayMilliseconds * 2;
                } else {
                    status = false;
                }
            }
        } while (numRetries-- > 0);
    }

    /*
     * Use a private function to reset the memory block instead of memset,
     * so that compiler wont optimize away the function call.
     */
    mem_clear(&xConnectParams, sizeof(WIFINetworkParams_t));

    return status;
}

static bool wifi_connect(void) {
    bool ret = true;
    if (network_state == NetworkStateDisabled) {
        if (WIFI_On() != eWiFiSuccess) {
            LogError(("wifi init failed"));
            return false;
        }
        network_state = NetworkStateEnabled;
        ret = connect_ap();
        if (ret) {
            network_state = NetworkStateConnected;
        }
    }
    return ret;
}

void start_ap() {
    struct wlan_network nw;
    wlan_initialize_uap_network(&nw);
    strncpy(nw.name, "ap", sizeof(nw.name));
    nw.type = WLAN_BSS_TYPE_UAP;
    nw.role = WLAN_BSS_ROLE_UAP;
    strncpy(nw.ssid, "sesame", sizeof(nw.ssid));
    nw.channel = 0;
    nw.ip.ipv4.addr_type = ADDR_TYPE_STATIC;
    FreeRTOS_inet_pton(FREERTOS_AF_INET, "192.168.4.1", &nw.ip.ipv4.address);
    FreeRTOS_inet_pton(FREERTOS_AF_INET, "255.255.255.0", &nw.ip.ipv4.netmask);
    FreeRTOS_inet_pton(FREERTOS_AF_INET, "192.168.4.1", &nw.ip.ipv4.gw);
    nw.security.type = WLAN_SECURITY_NONE;
    int ret = wlan_add_network(&nw);
    if (ret != WM_SUCCESS) {
        LogError(("network_mgr: failed to add network %d", ret));
        return;
    }

    int res = wlan_start_network("ap");
    if (res != WM_SUCCESS) {
        LogInfo(("failed to start AP: %d", res));
        return;
    }

    dhcp_task_params_t params = {nw.ip.ipv4.address, nw.ip.ipv4.netmask};
    xTaskCreate(dhcpd_task, "dhcpd", 512, &params, tskIDLE_PRIORITY, NULL);
}

static QueueHandle_t nm_queue;

void network_manager_task(void *params) {
    nm_queue = (QueueHandle_t)params;

    // if (WIFI_On() != eWiFiSuccess) {
    //     LogError(("wifi init failed"));
    //     goto err;
    // }

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

    wifi_connect();
    //  start_ap();

    for (;;) {
        vTaskDelay(1000);
        // nm_cmd_t cmd;
        // if (xQueueReceive(nm_queue, &cmd, portMAX_DELAY) == pdPASS) {
        //     switch (cmd) {
        //         case NM_CMD_AP_MODE:
        //             start_ap();
        //             break;
        //         default:
        //             LogError(("unknown nm_cmd_t %d", cmd));
        //     }
        // }
    }

err:
    vTaskDelete(NULL);
}
