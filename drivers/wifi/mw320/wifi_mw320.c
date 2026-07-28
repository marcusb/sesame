/* Remove overrides for Zephyr headers */
#undef hex2bin
#undef bin2hex
#undef wifi_scan_result

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wifi_mw320, LOG_LEVEL_DBG);

#include <stdio.h>
#include <stdarg.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/ethernet.h>
#include <zephyr/net/wifi_mgmt.h>

#include "fsl_sdioc.h"
#include "wifi_config.h"

#define wifi_scan_result mw320_sdk_wifi_scan_result
#include <wlan.h>
#include <wifi.h>
#undef wifi_scan_result

#define INTF_HEADER_LEN 4
#define TXPD_SIZE 22

/* Restore overrides for SDK headers */
#define hex2bin wm_hex2bin
#define bin2hex wm_bin2hex
#define wifi_scan_result mw320_wifi_scan_result

/* mw320_sdk includes */
#include "wlcmgr/wlan.h"

struct wifi_mw320_dev {
    struct net_if *iface;
    uint8_t mac_addr[6];
    int bss_type;
};

static struct wifi_mw320_dev mw320_data;

/* wm_net.h stubs for SDK compatibility */
void net_wlan_init(void) {}
int net_get_if_addr(struct wlan_ip_config *addr, void *intrfc_handle) { return 0; }
int net_get_if_ipv6_addr(struct wlan_ip_config *addr, void *intrfc_handle) { return 0; }
void net_configure_dns(struct wlan_ip_config *ip, enum wlan_bss_role role) {}
void net_interface_down(void *intrfc_handle) {
    if (mw320_data.iface) {
        net_eth_carrier_off(mw320_data.iface);
    }
}
void net_interface_dhcp_stop(void *intrfc_handle) {}
int net_configure_address(struct wlan_ip_config *addr, void *intrfc_handle) {
    int event = (mw320_data.bss_type == BSS_TYPE_STA) ? WIFI_EVENT_NET_STA_ADDR_CONFIG
                                                      : WIFI_EVENT_UAP_NET_ADDR_CONFIG;
    wlan_wlcmgr_send_msg(event, WIFI_EVENT_REASON_SUCCESS, NULL);
    return 0;
}
void *net_get_mlan_handle(void) { return NULL; }
void *net_get_uap_handle(void) { return NULL; }

static void wifi_mw320_iface_init(struct net_if *iface)
{
    struct wifi_mw320_dev *dev = net_if_get_device(iface)->data;
    dev->iface = iface;

    /* Read MAC address from wlan.h API and set it in Zephyr */
    wifi_mac_addr_t mac;
    if (wifi_get_device_mac_addr(&mac) == WM_SUCCESS) {
        memcpy(dev->mac_addr, mac.mac, 6);
    } else {
        LOG_ERR("Failed to get MAC address");
    }
    
    net_if_set_link_addr(iface, dev->mac_addr, 6, NET_LINK_ETHERNET);
    ethernet_init(iface);
    
    struct ethernet_context *eth_ctx = net_if_l2_data(iface);
    eth_ctx->eth_if_type = L2_ETH_IF_TYPE_WIFI;
}

static int wifi_mw320_send(const struct device *dev, struct net_pkt *pkt)
{
    const int pkt_len = net_pkt_get_len(pkt);
    uint32_t outbuf_len;
    uint8_t *outbuf = wifi_get_outbuf(&outbuf_len);

    if (!outbuf) {
        return -ENOBUFS;
    }

    uint8_t hdr_len = TXPD_SIZE + INTF_HEADER_LEN;
    if (hdr_len + pkt_len > outbuf_len) {
        return -EMSGSIZE;
    }

    memset(outbuf, 0, hdr_len);
    if (net_pkt_read(pkt, outbuf + hdr_len, pkt_len) < 0) {
        return -EIO;
    }

    int ret = wifi_low_level_output(mw320_data.bss_type, outbuf + hdr_len, pkt_len);
    if (ret != WM_SUCCESS) {
        return -EIO;
    }

    return 0;
}

static int wifi_mw320_mgmt_scan(const struct device *dev, struct net_if *iface,
                                struct wifi_scan_params *params,
                                scan_result_cb_t cb)
{
    // TODO: implement
    return -ENOTSUP;
}

static int wifi_mw320_mgmt_connect(const struct device *dev, struct net_if *iface,
                                   struct wifi_connect_req_params *params)
{
    struct wlan_network network;
    memset(&network, 0, sizeof(network));
    strcpy(network.name, "client");

    if (params->ssid_length > sizeof(network.ssid) - 1) {
        return -EINVAL;
    }
    memcpy(network.ssid, params->ssid, params->ssid_length);
    network.ssid[params->ssid_length] = '\0';

    if (params->security == WIFI_SECURITY_TYPE_PSK) {
        if (params->psk_length > sizeof(network.security.psk) - 1) {
            return -EINVAL;
        }
        memcpy(network.security.psk, params->psk, params->psk_length);
        network.security.psk[params->psk_length] = '\0';
        network.security.psk_len = params->psk_length;
        network.security.type = WLAN_SECURITY_WPA2;
    } else {
        network.security.type = WLAN_SECURITY_NONE;
    }

    network.type = WLAN_BSS_TYPE_STA;
    network.role = WLAN_BSS_ROLE_STA;
    /* We handle IP addressing via Zephyr's stack, not SDK's DHCP */
    network.ip.ipv4.addr_type = ADDR_TYPE_STATIC;
    network.ip.ipv4.address = 0;
    network.ip.ipv4.gw = 0;
    network.ip.ipv4.netmask = 0;

    wlan_remove_network(network.name);
    int res = wlan_add_network(&network);
    if (res != WM_SUCCESS) {
        LOG_ERR("wlan_add_network failed %d", res);
        return -EIO;
    }

    res = wlan_connect(network.name);
    if (res != WM_SUCCESS) {
        LOG_ERR("wlan_connect failed %d", res);
        return -EIO;
    }

    mw320_data.bss_type = BSS_TYPE_STA;
    return 0;
}

static int wifi_mw320_mgmt_disconnect(const struct device *dev, struct net_if *iface)
{
    int res = wlan_disconnect();
    if (res != WM_SUCCESS) {
        LOG_ERR("wlan_disconnect failed %d", res);
        return -EIO;
    }
    return 0;
}

static int wifi_mw320_mgmt_ap_enable(const struct device *dev, struct net_if *iface,
                                     struct wifi_connect_req_params *params)
{
    struct wlan_network network;
    memset(&network, 0, sizeof(network));
    strcpy(network.name, "uap");

    if (params->ssid_length > sizeof(network.ssid) - 1) {
        return -EINVAL;
    }
    memcpy(network.ssid, params->ssid, params->ssid_length);
    network.ssid[params->ssid_length] = '\0';

    if (params->security == WIFI_SECURITY_TYPE_PSK) {
        if (params->psk_length > sizeof(network.security.psk) - 1) {
            return -EINVAL;
        }
        memcpy(network.security.psk, params->psk, params->psk_length);
        network.security.psk[params->psk_length] = '\0';
        network.security.psk_len = params->psk_length;
        network.security.type = WLAN_SECURITY_WPA2;
    } else {
        network.security.type = WLAN_SECURITY_NONE;
    }

    network.type = WLAN_BSS_TYPE_UAP;
    network.role = WLAN_BSS_ROLE_UAP;
    /* We handle IP addressing via Zephyr's stack */
    network.ip.ipv4.addr_type = ADDR_TYPE_STATIC;
    network.ip.ipv4.address = 0;
    network.ip.ipv4.gw = 0;
    network.ip.ipv4.netmask = 0;

    wlan_remove_network(network.name);
    int res = wlan_add_network(&network);
    if (res != WM_SUCCESS) {
        LOG_ERR("wlan_add_network (uap) failed %d", res);
        return -EIO;
    }

    res = wlan_start_network(network.name);
    if (res != WM_SUCCESS) {
        LOG_ERR("wlan_start_network failed %d", res);
        wifi_mgmt_raise_ap_enable_result_event(iface, WIFI_STATUS_AP_FAIL);
        return -EIO;
    }

    mw320_data.bss_type = BSS_TYPE_UAP;
    wifi_mgmt_raise_ap_enable_result_event(iface, WIFI_STATUS_AP_SUCCESS);
    return 0;
}

static int wifi_mw320_mgmt_ap_disable(const struct device *dev, struct net_if *iface)
{
    int res = wlan_stop_network("uap");
    if (res != WM_SUCCESS) {
        LOG_ERR("wlan_stop_network failed %d", res);
        wifi_mgmt_raise_ap_disable_result_event(iface, WIFI_STATUS_AP_FAIL);
        return -EIO;
    }
    wifi_mgmt_raise_ap_disable_result_event(iface, WIFI_STATUS_AP_SUCCESS);
    return 0;
}

static const struct wifi_mgmt_ops wifi_mw320_mgmt_ops = {
    .scan       = wifi_mw320_mgmt_scan,
    .connect    = wifi_mw320_mgmt_connect,
    .disconnect = wifi_mw320_mgmt_disconnect,
    .ap_enable  = wifi_mw320_mgmt_ap_enable,
    .ap_disable = wifi_mw320_mgmt_ap_disable,
};

static int wifi_mw320_set_config(const struct device *dev,
				 struct net_if *iface,
				 enum ethernet_config_type type,
				 const struct ethernet_config *config)
{
	if (type == ETHERNET_CONFIG_TYPE_FILTER) {
		if (config->filter.type == ETHERNET_FILTER_TYPE_DST_MAC_ADDRESS) {
			if (config->filter.set) {
				wifi_add_mcast_filter((uint8_t *)config->filter.mac_address.addr);
			} else {
				wifi_remove_mcast_filter((uint8_t *)config->filter.mac_address.addr);
			}
			return 0;
		}
	}
	return -ENOTSUP;
}

static enum ethernet_hw_caps wifi_mw320_get_capabilities(const struct device *dev, struct net_if *iface)
{
	return ETHERNET_HW_FILTERING;
}

static const struct net_wifi_mgmt_offload wifi_mw320_api = {
    .wifi_iface.iface_api.init = wifi_mw320_iface_init,
    .wifi_iface.get_capabilities = wifi_mw320_get_capabilities,
    .wifi_iface.send = wifi_mw320_send,
    .wifi_iface.set_config = wifi_mw320_set_config,
    .wifi_mgmt_api = &wifi_mw320_mgmt_ops,
};

static int wifi_mw320_event_callback(enum wlan_event_reason event, void *data)
{
    struct wifi_mw320_dev *dev = &mw320_data;

    switch (event) {
    case WLAN_REASON_INITIALIZED:
        LOG_INF("WLAN Initialized");
        /* wlan_init and wlan_start succeeded. Now we should add network and connect? */
        /* Zephyr manages network connections via mgmt API, so we just report interface up. */
        if (dev->iface) {
            net_if_up(dev->iface);
            net_eth_carrier_on(dev->iface);
        }
        break;
    case WLAN_REASON_SUCCESS:
        LOG_INF("WLAN Connected");
        if (dev->iface) {
            wifi_mgmt_raise_connect_result_event(dev->iface, 0);
        }
        break;
    case WLAN_REASON_CONNECT_FAILED:
    case WLAN_REASON_NETWORK_NOT_FOUND:
    case WLAN_REASON_NETWORK_AUTH_FAILED:
        LOG_ERR("WLAN Connect Failed: %d", event);
        if (dev->iface) {
            wifi_mgmt_raise_connect_result_event(dev->iface, -1);
        }
        break;
    case WLAN_REASON_ADDRESS_FAILED:
    case WLAN_REASON_LINK_LOST:
    case WLAN_REASON_USER_DISCONNECT:
        LOG_INF("WLAN Disconnected: %d", event);
        if (dev->iface) {
            wifi_mgmt_raise_disconnect_result_event(dev->iface, 0);
        }
        break;
    default:
        LOG_DBG("WLAN Event: %d", event);
        break;
    }
    return 0;
}


#include <zephyr/irq.h>
#include <stdbool.h>
#include "mflash_drv.h"

__ramfunc static void * wifi_mw320_get_fw_ptr(void)
{
    uint32_t offset = 0;
    if ((FLASHC->FCACR & FLASHC_FCACR_OFFSET_EN_MASK) != 0U) {
        offset = FLASHC->FAOFFR;
    }
    
    // The physical address of the Wi-Fi firmware from Device Tree
    uint32_t phys_addr = DT_REG_ADDR(DT_NODELABEL(wififw_partition));
    
    // Map it to logical address space
    return (void *)(MFLASH_BASE_ADDRESS + phys_addr - offset);
}

#include <mlan_api.h>
#include <wifi-internal.h>

static void wifi_mw320_data_input_callback(const uint8_t interface, const uint8_t *buffer, const uint16_t len)
{
    struct wifi_mw320_dev *dev = &mw320_data;
    if (!dev->iface) return;

    RxPD * rxpd = (RxPD *)(buffer + INTF_HEADER_LEN);
    
    if (rxpd->rx_pkt_type == PKT_TYPE_AMSDU) {
        LOG_WRN("AMSDU not supported in Zephyr port yet");
        return;
    }
    
    uint8_t *payload = (uint8_t *)rxpd + rxpd->rx_pkt_offset;
    uint16_t payload_len = rxpd->rx_pkt_length;
    
    struct net_pkt *pkt = net_pkt_rx_alloc_with_buffer(dev->iface, payload_len, AF_UNSPEC, 0, K_NO_WAIT);
    if (!pkt) {
        LOG_ERR("Failed to allocate RX net_pkt");
        return;
    }
    
    static const uint8_t rfc1042_eth_hdr[] = { 0xaa, 0xaa, 0x03, 0x00, 0x00, 0x00 };
    if (!memcmp(payload + sizeof(struct net_eth_hdr), rfc1042_eth_hdr, sizeof(rfc1042_eth_hdr))) {
        struct net_eth_hdr ethhdr;
        memcpy(&ethhdr, payload, sizeof(ethhdr));
        ethhdr.type = *(uint16_t *)(payload + sizeof(struct net_eth_hdr) + sizeof(rfc1042_eth_hdr));
        
        if (net_pkt_write(pkt, &ethhdr, sizeof(ethhdr)) < 0) {
            printk("<err> wifi_mw320: Failed to write to net_pkt\n");
            net_pkt_unref(pkt);
            return;
        }
        if (net_pkt_write(pkt, payload + sizeof(struct net_eth_hdr) + 8, payload_len - sizeof(struct net_eth_hdr) - 8) < 0) {
            printk("<err> wifi_mw320: Failed to write to net_pkt\n");
            net_pkt_unref(pkt);
            return;
        }
    } else {
        if (net_pkt_write(pkt, payload, payload_len) < 0) {
            printk("<err> wifi_mw320: Failed to write to net_pkt\n");
            net_pkt_unref(pkt);
            return;
        }
    }

    net_pkt_cursor_init(pkt);
    if (net_recv_data(dev->iface, pkt) < 0) {
        LOG_ERR("net_recv_data failed");
        net_pkt_unref(pkt);
    }
}

static bool wifi_mw320_is_ip_or_ipv6(const uint8_t *buffer)
{
    /* Always return false to disable AMPDU. The FreeRTOS implementation
     * seems to have an endianness bug here that caused it to always return
     * false, preventing ADDBA requests from being sent and avoiding AP
     * disconnects. */
    return false;
}

static int wifi_mw320_init(const struct device *dev)
{
    int ret;

    void *fw_ptr = wifi_mw320_get_fw_ptr();
    if (!fw_ptr) {
        LOG_ERR("wifi_mw320_get_fw_ptr returned NULL");
        return -ENODEV;
    }

    LOG_INF("WLAN FW found at offset 0x%x", (uint32_t)fw_ptr);
    
    uint32_t *wififw = (uint32_t *)fw_ptr;
    
    uint32_t magic = *wififw;
    LOG_INF("Found wififw at %p, magic: 0x%08x", wififw, magic);
    
    if (magic != (('W' << 0) | ('L' << 8) | ('F' << 16) | ('W' << 24))) {
        LOG_ERR("WiFi firmware missing or invalid magic!");
        return -ENODEV;
    }

    /* Connect SDIO Interrupt for the WiFi driver */
    extern void SDIO_DriverIRQHandler(void);
    IRQ_CONNECT(28 /* SDIO_IRQn */, 5, SDIO_DriverIRQHandler, NULL, 0);
    irq_enable(28);

    /* Initialize WIFI Driver */
    LOG_INF("SDIOC_GetPresentStatus = 0x%08x", SDIOC_GetPresentStatus(SDIOC));

    wifi_register_wrapper_net_is_ip_or_ipv6_callback(wifi_mw320_is_ip_or_ipv6);
    wifi_register_data_input_callback(wifi_mw320_data_input_callback);

    LOG_INF("loading wlan firmware");
    ret = wlan_init((const uint8_t *)(wififw + 2U), *(wififw + 1U));
    if (ret != WM_SUCCESS) {
        LOG_ERR("wlan_init failed: %d", ret);
        return -EIO;
    }
    LOG_INF("wlan initialized");

    ret = wlan_start(wifi_mw320_event_callback);
    if (ret != WM_SUCCESS) {
        LOG_ERR("wlan_start failed: %d", ret);
        return -EIO;
    }
    
    return 0;
}

NET_DEVICE_INIT(wifi_mw320, "WIFI_MW320",
                wifi_mw320_init, NULL,
                &mw320_data, NULL,
                CONFIG_WIFI_INIT_PRIORITY,
                &wifi_mw320_api,
                ETHERNET_L2,
                NET_L2_GET_CTX_TYPE(ETHERNET_L2),
                NET_ETH_MTU);


