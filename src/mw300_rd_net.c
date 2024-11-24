/**
 * MW300 network interface driver. Supports both STA and UAP interfaces.
 */

#include <stdlib.h>
#include <string.h>

/* FreeRTOS includes. */
#include "FreeRTOS.h"
#include "list.h"

/* FreeRTOS+TCP includes. */
#include "FreeRTOS_IP.h"

/* FreeRTOS+TCP includes. */
#include "FreeRTOS_IP.h"
#include "FreeRTOS_IP_Private.h"
#include "FreeRTOS_Sockets.h"
#include "NetworkBufferManagement.h"
#include "NetworkInterface.h"
#include "mw300_rd_net.h"

/* wmsdk includes */
#include "wifi-decl.h"
#include "wifi.h"
#include "wm_net.h"
#include "wmerrno.h"
#include "wmlog.h"

#define net_e(...) wmlog_e("freertos_tcp", ##__VA_ARGS__)
#define net_w(...) wmlog_w("freertos_tcp", ##__VA_ARGS__)
#define net_d(...) wmlog("freertos_tcp", ##__VA_ARGS__)

// global output buffer provided by the WiFi driver
extern uint8_t outbuf[2048];

uint8_t* mlan_get_payload(const uint8_t* rcvdata, uint16_t* payload_len,
                          int* interface);

#define NUM_INTERFACES 2
static NetworkInterface_t* interfaces[NUM_INTERFACES];
_Static_assert(BSS_TYPE_STA < NUM_INTERFACES, "ifindex");
_Static_assert(BSS_TYPE_UAP < NUM_INTERFACES, "ifindex");

/*******************************************************
 *  wlan driver callbacks
 *******************************************************/

void net_interface_down(void* intrfc_handle) {
    net_d("net_interface_down\r\n");
    NetworkInterface_t* iface = (NetworkInterface_t*)intrfc_handle;
    FreeRTOS_NetworkDown(iface);
}

void* net_get_mlan_handle() { return interfaces[BSS_TYPE_STA]; }

void* net_get_uap_handle(void) { return interfaces[BSS_TYPE_UAP]; }

int net_get_if_addr(struct wlan_ip_config* addr, void* intrfc_handle) {
    NetworkInterface_t* iface = (NetworkInterface_t*)intrfc_handle;
    NetworkEndPoint_t* ep = FreeRTOS_FirstEndPoint(iface);
    if (ep != NULL) {
        addr->ipv4.address = ep->ipv4_settings.ulIPAddress;
        addr->ipv4.netmask = ep->ipv4_settings.ulNetMask;
        addr->ipv4.gw = ep->ipv4_settings.ulGatewayAddress;
        addr->ipv4.dns1 = ep->ipv4_settings.ulDNSServerAddresses[0];
        addr->ipv4.dns2 = ep->ipv4_settings.ulDNSServerAddresses[1];
        return WM_SUCCESS;
    } else {
        return WM_FAIL;
    }
}

void handle_data_packet(uint8_t interface, const uint8_t* buffer,
                        uint16_t len) {
    t_u16 payload_len;
    int interface_i = interface;
    uint8_t* payload = mlan_get_payload(buffer, &payload_len, &interface_i);

    if (eConsiderFrameForProcessing(payload) != eProcessBuffer) {
        net_d("Dropping packet\r\n");
        return;
    }

    if (interface >= NUM_INTERFACES) {
        net_e("invalid interface %d\n", interface);
    }

    const TickType_t xDescriptorWaitTime = pdMS_TO_TICKS(250);
    NetworkBufferDescriptor_t* pxNetworkBuffer =
        pxGetNetworkBufferWithDescriptor(len, xDescriptorWaitTime);

    if (pxNetworkBuffer != NULL) {
        pxNetworkBuffer->xDataLength = payload_len;
        memcpy(pxNetworkBuffer->pucEthernetBuffer, payload, payload_len);

        pxNetworkBuffer->pxInterface = interfaces[interface];
        pxNetworkBuffer->pxEndPoint = FreeRTOS_MatchingEndpoint(
            pxNetworkBuffer->pxInterface, pxNetworkBuffer->pucEthernetBuffer);

        IPStackEvent_t xRxEvent = {eNetworkRxEvent, pxNetworkBuffer};

        if (xSendEventStructToIPTask(&xRxEvent, xDescriptorWaitTime) ==
            pdFAIL) {
            wmprintf("Failed to enqueue packet to network stack %p, len %d",
                     payload, payload_len);
            vReleaseNetworkBufferAndDescriptor(pxNetworkBuffer);
            return;
        }
    }
}

void net_wlan_init(void) {
    static int net_wlan_init_done;

    if (!net_wlan_init_done) {
        wifi_register_data_input_callback(&handle_data_packet);
        wlan_wlcmgr_send_msg(WIFI_EVENT_NET_INTERFACE_CONFIG,
                             WIFI_EVENT_REASON_SUCCESS, NULL);
        net_wlan_init_done = 1;
        net_d("Initialized wlan driver");
    }
}

int net_configure_address(struct wlan_ip_config* addr, void* intrfc_handle) {
    if (addr == NULL || intrfc_handle == NULL) {
        return -WM_E_INVAL;
    }

    wlan_wlcmgr_send_msg(WIFI_EVENT_NET_STA_ADDR_CONFIG,
                         WIFI_EVENT_REASON_SUCCESS, NULL);
    return 0;
}

void net_configure_dns(struct wlan_ip_config* ip, enum wlan_bss_role role) {}

void net_interface_dhcp_stop(void* intrfc_handle) {}

/*******************************************************
 *  Application API
 *******************************************************/

void notify_dhcp_configured() {
    wlan_wlcmgr_send_msg(WIFI_EVENT_NET_DHCP_CONFIG, WIFI_EVENT_REASON_SUCCESS,
                         NULL);
}

void vNetworkInterfaceAllocateRAMToBuffers(
    NetworkBufferDescriptor_t
        pxNetworkBuffers[ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS]) {}

static BaseType_t xMW300_NetworkInterfaceInitialise(NetworkInterface_t* iface) {
    uint8_t if_index = (int)iface->pvArgument;
    net_d("NET INITIALIZE IFACE %u ret %d\r\n", if_index,
          iface->bits.bInterfaceUp);
    configASSERT(if_index < NUM_INTERFACES);
    interfaces[if_index] = iface;

    mac_addr_t mac_addr;
    int ret = wifi_get_device_mac_addr(&mac_addr);
    if (ret != WM_SUCCESS) {
        net_d("Failed to get mac address");
        return pdFALSE;
    }
    NetworkEndPoint_t* ep = FreeRTOS_FirstEndPoint(iface);
    if (ep != NULL) {
        memcpy(ep->xMACAddress.ucBytes, mac_addr.mac,
               sizeof(ep->xMACAddress.ucBytes));
    }

    enum wlan_connection_state state;
    switch (if_index) {
        case BSS_TYPE_STA:

            ret = wlan_get_connection_state(&state);
            break;
        case BSS_TYPE_UAP:
            ret = wlan_get_uap_connection_state(&state);
            break;
        default:
            ret = -WM_E_INVAL;
    }
    if (ret != WM_SUCCESS) {
        net_e("wlan get connection state failed\r\n");
        return pdFALSE;
    }
    net_d("wlan_connection_state %d\r\n", state);
    return state == WLAN_ASSOCIATED || state == WLAN_CONNECTING ||
           state == WLAN_CONNECTED || state == WLAN_UAP_STARTED;
}

static BaseType_t xMW300_GetPhyLinkStatus(NetworkInterface_t* iface) {
    return iface->bits.bInterfaceUp;
}

static BaseType_t xMW300_NetworkInterfaceOutput(
    NetworkInterface_t* iface, NetworkBufferDescriptor_t* const pxNetworkBuffer,
    BaseType_t xReleaseAfterSend) {
    if ((pxNetworkBuffer == NULL) ||
        (pxNetworkBuffer->pucEthernetBuffer == NULL) ||
        (pxNetworkBuffer->xDataLength == 0)) {
        net_d("Incorrect params");
        return pdFALSE;
    }

    memset(outbuf, 0x00, sizeof(outbuf));
    uint8_t pkt_len = 22 + 4; /* sizeof(TxPD) + INTF_HEADER_LEN */
    memcpy(outbuf + pkt_len, pxNetworkBuffer->pucEthernetBuffer,
           pxNetworkBuffer->xDataLength);
    uint8_t if_idx = (int)iface->pvArgument;
    int ret = wifi_low_level_output(if_idx, outbuf + pkt_len,
                                    pxNetworkBuffer->xDataLength);

    if (ret != WM_SUCCESS) {
        net_e("Failed output %p, length %d, error %d \r\n",
              pxNetworkBuffer->pucEthernetBuffer, pxNetworkBuffer->xDataLength,
              ret);
    }

    if (xReleaseAfterSend != pdFALSE) {
        vReleaseNetworkBufferAndDescriptor(pxNetworkBuffer);
    }

    return ret == WM_SUCCESS ? pdTRUE : pdFALSE;
}

NetworkInterface_t* pxMW300_FillInterfaceDescriptor(
    uint8_t if_type, NetworkInterface_t* pxInterface) {
    memset(pxInterface, 0, sizeof(*pxInterface));
    switch (if_type) {
        case BSS_TYPE_STA:
            pxInterface->pcName = "wl0";
            break;
        case BSS_TYPE_UAP:
            pxInterface->pcName = "ap0";
            break;
        default:
            net_e("invalid if_type %u", if_type);
            return NULL;
    }

    pxInterface->pvArgument = if_type;
    pxInterface->pfInitialise = xMW300_NetworkInterfaceInitialise;
    pxInterface->pfOutput = xMW300_NetworkInterfaceOutput;
    pxInterface->pfGetPhyLinkStatus = xMW300_GetPhyLinkStatus;

    FreeRTOS_AddNetworkInterface(pxInterface);

    return pxInterface;
}
