/* Remove overrides for Zephyr headers */
#undef hex2bin
#undef bin2hex
#undef wifi_scan_result

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wifi_mw320, CONFIG_WIFI_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
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
int net_configure_address(struct wlan_ip_config *addr, void *intrfc_handle) { return 0; }
void *net_get_mlan_handle(void) { return NULL; }
void *net_get_uap_handle(void) { return NULL; }

static void wifi_mw320_iface_init(struct net_if *iface)
{
    struct wifi_mw320_dev *dev = net_if_get_device(iface)->data;
    dev->iface = iface;

    /* Read MAC address from wlan.h API and set it in Zephyr */
    /* wlan_get_mac_address(dev->mac_addr); */
    
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

    int ret = wifi_low_level_output(BSS_TYPE_STA, outbuf + hdr_len, pkt_len);
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
        return -EIO;
    }

    return 0;
}

static int wifi_mw320_mgmt_ap_disable(const struct device *dev, struct net_if *iface)
{
    int res = wlan_stop_network("uap");
    if (res != WM_SUCCESS) {
        LOG_ERR("wlan_stop_network failed %d", res);
        return -EIO;
    }
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

static const struct net_wifi_mgmt_offload wifi_mw320_api = {
    .wifi_iface.iface_api.init = wifi_mw320_iface_init,
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



#define FL_PART1_START 0x4000
#define FL_PART2_START 0x5000
#include "../../../mw320_sdk/components/boot2_utils/partition.h"

struct fw_ptr_debug_info {
    uint32_t fw_ptr;
    uint32_t t1_magic;
    uint32_t t2_magic;
    uint32_t num_entries;
    struct partition_entry entries[10];
};

__ramfunc static void wifi_mw320_get_fw_ptr(struct fw_ptr_debug_info *dbg)
{
    unsigned int key = irq_lock();
    uint32_t offset = 0;
    
    uint32_t old_fcacr = FLASHC->FCACR;
    uint32_t old_faoffr = FLASHC->FAOFFR;
    
    if ((FLASHC->FCACR & FLASHC_FCACR_OFFSET_EN_MASK) != 0U) {
        offset = FLASHC->FAOFFR;
    }
    
    // Instead of disabling offset, just set it to 0 and flush cache
    FLASHC->FAOFFR = 0;
    
    // Flush cache so we read from physical 0
    FLASHC->FCCR |= FLASHC_FCCR_CACHE_LINE_FLUSH_MASK;
    while (FLASHC->FCCR & FLASHC_FCCR_CACHE_LINE_FLUSH_MASK) {}
    
    // Read directly from physical flash using memory mapping
    struct partition_table t1, t2;
    uint32_t *dst = (uint32_t *)&t1;
    uint32_t *src = (uint32_t *)(0x1F000000 + 0x4000);
    uint32_t t1_words = sizeof(t1) >> 2;
    for (uint32_t i=0; i<t1_words; i++) dst[i] = src[i];
    
    dst = (uint32_t *)&t2;
    src = (uint32_t *)(0x1F000000 + 0x5000);
    uint32_t t2_words = sizeof(t2) >> 2;
    for (uint32_t i=0; i<t2_words; i++) dst[i] = src[i];
    
    dbg->t1_magic = t1.magic;
    dbg->t2_magic = t2.magic;
    
    struct partition_table *active_t = NULL;
    bool t1_valid = (t1.magic == PARTITION_TABLE_MAGIC);
    bool t2_valid = (t2.magic == PARTITION_TABLE_MAGIC);
    
    if (t1_valid && t2_valid) {
        if (t1.gen_level >= t2.gen_level) {
            active_t = &t1;
        } else {
            active_t = &t2;
        }
    } else if (t1_valid) {
        active_t = &t1;
    } else if (t2_valid) {
        active_t = &t2;
    }
    
    uint32_t active_fw_start = 0;
    dbg->num_entries = 0;
    if (active_t) {
        uint32_t active_t_addr = (active_t == &t1) ? 0x4000 : 0x5000;
        uint32_t entries_addr = active_t_addr + sizeof(struct partition_table);
        
        uint32_t num_entries = active_t->partition_entries_no;
        if (num_entries > 10) num_entries = 10;
        dbg->num_entries = num_entries;
        
        dst = (uint32_t *)dbg->entries;
        src = (uint32_t *)(0x1F000000 + entries_addr);
        uint32_t entry_words = (sizeof(struct partition_entry) * num_entries) >> 2;
        for (uint32_t i=0; i<entry_words; i++) dst[i] = src[i];
        
        struct partition_entry *active_fw = NULL;
        for (int i = 0; i < num_entries; i++) {
            if (dbg->entries[i].type == FC_COMP_WLAN_FW) {
                if (!active_fw || dbg->entries[i].gen_level > active_fw->gen_level) {
                    active_fw = &dbg->entries[i];
                }
            }
        }
        if (active_fw) {
            active_fw_start = active_fw->start;
        }
    }
    
    // Restore offset
    FLASHC->FAOFFR = old_faoffr;
    FLASHC->FCACR = old_fcacr;
    
    // Flush cache again to ensure instructions are fetched correctly with the new offset
    FLASHC->FCCR |= FLASHC_FCCR_CACHE_LINE_FLUSH_MASK;
    while (FLASHC->FCCR & FLASHC_FCCR_CACHE_LINE_FLUSH_MASK) {}
    
    irq_unlock(key);
    
    dbg->fw_ptr = 0;
    if (active_fw_start) {
        offset = 0;
        if ((FLASHC->FCACR & FLASHC_FCACR_OFFSET_EN_MASK) != 0U) {
            offset = FLASHC->FAOFFR;
        }
        dbg->fw_ptr = 0x1F000000 + active_fw_start - offset;
    }
}


static bool wifi_mw320_is_ip_or_ipv6(const uint8_t *buffer)
{
    struct net_eth_hdr *hdr = (struct net_eth_hdr *)buffer;
    uint16_t type = ntohs(hdr->type);
    return (type == NET_ETH_PTYPE_IP || type == NET_ETH_PTYPE_IPV6);
}

static int wifi_mw320_init(const struct device *dev)
{
    int ret;
    struct wifi_mw320_config *cfg = (struct wifi_mw320_config *)dev->config;

    struct fw_ptr_debug_info dbg = {0};
    wifi_mw320_get_fw_ptr(&dbg);
    
    void *fw_ptr = (void *)dbg.fw_ptr;
    if (!fw_ptr) {
        LOG_ERR("wifi_mw320_get_fw_ptr returned NULL");
        
        // Debug read from flash using memory map to see what we actually got
        uint32_t *magic = (uint32_t *)(0x1F000000 + 0x4000);
        LOG_ERR("Direct read from 0x1F004000: 0x%08x", *magic);
        
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
    

    
    extern int wifi_register_wrapper_net_is_ip_or_ipv6_callback(bool (*)(const uint8_t *));
    wifi_register_wrapper_net_is_ip_or_ipv6_callback(wifi_mw320_is_ip_or_ipv6);

    LOG_INF("Calling wlan_init...");
    ret = wlan_init((const uint8_t *)(wififw + 2U), *(wififw + 1U));
    LOG_INF("wlan_init returned %d", ret);
    if (ret != WM_SUCCESS) {
        LOG_ERR("wlan_init failed: %d", ret);
        return -EIO;
    }

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

