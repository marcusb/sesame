#include <stdbool.h>
#include <string.h>

#include "FreeRTOS.h"
#include "app_logging.h"
#include "iot_wifi.h"
#include "wlan.h"

typedef enum NetworkState {
    NetworkStateDisabled = 0,
    NetworkStateEnabled = 1,
    NetworkStateConnected = 2,
} NetworkState_t;

NetworkState_t network_state;

/**
 * @brief Function to set a memory block to zero.
 * The function sets memory to zero using a volatile pointer so that compiler
 * wont optimize out the function if the buffer to be set to zero is not used
 * further.
 *
 * @param pBuf Pointer to buffer to be set to zero
 * @param size Length of the buffer to be set zero
 */
static void prvMemzero(void *pBuf, size_t size) {
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
        /* Try to connect to wifi access point with retry and exponential delay
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
     * Use a private function to reset the memory block instead of memset, so
     * that compiler wont optimize away the function call.
     */
    prvMemzero(&xConnectParams, sizeof(WIFINetworkParams_t));

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

void network_manager_task(void *const params) {
    wifi_connect();

    while (true) {
        vTaskDelay(1000);
    }
}
