/**
 * MW300 network interface driver. Supports both STA and UAP interfaces.
 */

#include <stdlib.h>
#include <string.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_IP_Private.h"
#include "FreeRTOS_Sockets.h"
#include "NetworkBufferManagement.h"
#include "NetworkInterface.h"
#include "list.h"
#include "mw300_rd_net.h"

// mw320
#include "mlan_api.h"
#include "mlan_fw.h"
#include "wifi-sdio.h"
#include "wm_net.h"
#include "wmlog.h"

#define net_e(...) wmlog_e("net", ##__VA_ARGS__)
#define net_w(...) wmlog_w("net", ##__VA_ARGS__)
#define net_d(...) wmlog("net", ##__VA_ARGS__)

#define NUM_INTERFACES 2
static NetworkInterface_t* interfaces[NUM_INTERFACES];
_Static_assert(BSS_TYPE_STA < NUM_INTERFACES, "ifindex");
_Static_assert(BSS_TYPE_UAP < NUM_INTERFACES, "ifindex");

/*******************************************************
 *  wlan driver callbacks
 *******************************************************/

void net_interface_down(void* intrfc_handle) {
    NetworkInterface_t* netif = (NetworkInterface_t*)intrfc_handle;
    FreeRTOS_NetworkDown(netif);
}

void* net_get_mlan_handle() { return interfaces[BSS_TYPE_STA]; }

void* net_get_uap_handle(void) { return interfaces[BSS_TYPE_UAP]; }

int net_get_if_addr(struct wlan_ip_config* addr, void* intrfc_handle) {
    NetworkInterface_t* netif = (NetworkInterface_t*)intrfc_handle;
    NetworkEndPoint_t* ep = FreeRTOS_FirstEndPoint(netif);
    if (ep == NULL) {
        return WM_FAIL;
    }
    addr->ipv4.address = ep->ipv4_settings.ulIPAddress;
    addr->ipv4.netmask = ep->ipv4_settings.ulNetMask;
    addr->ipv4.gw = ep->ipv4_settings.ulGatewayAddress;
    addr->ipv4.dns1 = ep->ipv4_settings.ulDNSServerAddresses[0];
    addr->ipv4.dns2 = ep->ipv4_settings.ulDNSServerAddresses[1];
    return WM_SUCCESS;
}

static void deliver_packet_above(uint8_t iface, const uint8_t* data,
                                 const uint16_t len) {
    NetworkBufferDescriptor_t* buf =
        pxGetNetworkBufferWithDescriptor(len, pdMS_TO_TICKS(250));
    if (buf == NULL) {
        return;
    }
    buf->xDataLength = len;
    memcpy(buf->pucEthernetBuffer, data, len);
    buf->pxInterface = interfaces[iface];
    buf->pxEndPoint =
        FreeRTOS_MatchingEndpoint(buf->pxInterface, buf->pucEthernetBuffer);

    IPStackEvent_t xRxEvent = {eNetworkRxEvent, buf};
    if (xSendEventStructToIPTask(&xRxEvent, pdMS_TO_TICKS(250)) == pdFAIL) {
        net_w("failed to enqueue packet to network stack");
        vReleaseNetworkBufferAndDescriptor(buf);
        return;
    }
}

static void handle_data_packet(const uint8_t interface, const uint8_t* data,
                               const uint16_t len) {
    if (interface >= NUM_INTERFACES || interfaces[interface] == NULL) {
        return;
    }
    RxPD* rxpd = (RxPD*)((t_u8*)data + INTF_HEADER_LEN);
    if (rxpd->rx_pkt_type == PKT_TYPE_AMSDU) {
        wrapper_wlan_handle_amsdu_rx_packet(data, len);
        return;
    }

    const uint8_t* payload = (const uint8_t*)rxpd + rxpd->rx_pkt_offset;
    if (eConsiderFrameForProcessing(payload) != eProcessBuffer) {
        return;
    }

    deliver_packet_above(interface, payload, rxpd->rx_pkt_length);
}

static void handle_amsdu_data_packet(uint8_t interface, uint8_t* rcvdata,
                                     uint16_t datalen) {
    deliver_packet_above(interface, rcvdata, datalen);
}

static void handle_deliver_packet_above(uint8_t interface, void* lwip_pbuf) {
    // struct pbuf* p = (struct pbuf*)lwip_pbuf;
    // deliver_packet_above(p, interface);
    net_e("handle_deliver_packet_above()");
}

static inline bool wrapper_net_is_ip_or_ipv6(const uint8_t* buffer) {
    const EthernetHeader_t* eth = (const EthernetHeader_t*)buffer;
    return eth->usFrameType == ipIPv4_FRAME_TYPE ||
           eth->usFrameType == ipIPv6_FRAME_TYPE;
}

void net_wlan_init(void) {
    static int net_wlan_init_done;

    if (!net_wlan_init_done) {
        wifi_register_data_input_callback(&handle_data_packet);
        wifi_register_amsdu_data_input_callback(&handle_amsdu_data_packet);
        wifi_register_deliver_packet_above_callback(
            &handle_deliver_packet_above);
        wifi_register_wrapper_net_is_ip_or_ipv6_callback(
            &wrapper_net_is_ip_or_ipv6);
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
    NetworkInterface_t* netif = (NetworkInterface_t*)intrfc_handle;
    uint8_t if_index = (int)netif->pvArgument;
    if (if_index == BSS_TYPE_STA) {
        wlan_wlcmgr_send_msg(WIFI_EVENT_NET_STA_ADDR_CONFIG,
                             WIFI_EVENT_REASON_SUCCESS, NULL);
    } else if (if_index == BSS_TYPE_UAP) {
        wlan_wlcmgr_send_msg(WIFI_EVENT_UAP_NET_ADDR_CONFIG,
                             WIFI_EVENT_REASON_SUCCESS, NULL);
    }
    return WM_SUCCESS;
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

static BaseType_t netif_init(NetworkInterface_t* netif) {
    uint8_t if_index = (int)netif->pvArgument;
    configASSERT(if_index < NUM_INTERFACES);
    interfaces[if_index] = netif;

    wifi_mac_addr_t mac_addr;
    int ret = wifi_get_device_mac_addr(&mac_addr);
    if (ret != WM_SUCCESS) {
        net_d("Failed to get mac address");
        return pdFALSE;
    }
    NetworkEndPoint_t* ep = FreeRTOS_FirstEndPoint(netif);
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
        // wlan may not be initialized yet
        return pdFALSE;
    }
    return state == WLAN_ASSOCIATED || state == WLAN_CONNECTING ||
           state == WLAN_CONNECTED || state == WLAN_UAP_STARTED;
}

static BaseType_t get_phy_link_status(NetworkInterface_t* netif) {
    return netif->bits.bInterfaceUp;
}

static BaseType_t low_level_output(NetworkInterface_t* netif,
                                   NetworkBufferDescriptor_t* const buf,
                                   BaseType_t release_after_send) {
    if ((buf == NULL) || (buf->pucEthernetBuffer == NULL) ||
        (buf->xDataLength == 0)) {
        net_w("incorrect params");
        return pdFALSE;
    }

    uint32_t outbuf_len;
    uint8_t* outbuf = wifi_get_outbuf(&outbuf_len);
    if (!outbuf) {
        return pdFALSE;
    }
    uint8_t pkt_len = sizeof(TxPD) + INTF_HEADER_LEN;
    configASSERT(pkt_len + buf->xDataLength <= outbuf_len);
    memset(outbuf, 0x00, pkt_len);
    memcpy(outbuf + pkt_len, buf->pucEthernetBuffer, buf->xDataLength);

    uint8_t if_idx = (int)netif->pvArgument;
    int ret = wifi_low_level_output(if_idx, outbuf + pkt_len, buf->xDataLength);
    if (ret != WM_SUCCESS) {
        net_e("Failed output %p, length %d, error %d \r\n",
              buf->pucEthernetBuffer, buf->xDataLength, ret);
    }

    if (release_after_send != pdFALSE) {
        vReleaseNetworkBufferAndDescriptor(buf);
    }

    return ret == WM_SUCCESS ? pdTRUE : pdFALSE;
}

NetworkInterface_t* mw300_new_netif_desc(uint8_t if_type,
                                         NetworkInterface_t* netif) {
    memset(netif, 0, sizeof(*netif));
    switch (if_type) {
        case BSS_TYPE_STA:
            netif->pcName = "wl0";
            break;
        case BSS_TYPE_UAP:
            netif->pcName = "ap0";
            break;
        default:
            net_e("invalid if_type %u", if_type);
            return NULL;
    }

    netif->pvArgument = (void*)(int)if_type;
    netif->pfInitialise = netif_init;
    netif->pfOutput = low_level_output;
    netif->pfGetPhyLinkStatus = get_phy_link_status;

    FreeRTOS_AddNetworkInterface(netif);

    return netif;
}
