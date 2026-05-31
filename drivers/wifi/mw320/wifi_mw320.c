#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(wifi_mw320, CONFIG_WIFI_LOG_LEVEL);

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/net/wifi_mgmt.h>

/* mw320_sdk includes */
#include "wlcmgr/wlan.h"

struct wifi_mw320_dev {
    struct net_if *iface;
    uint8_t mac_addr[6];
};

static struct wifi_mw320_dev mw320_data;

static void wifi_mw320_iface_init(struct net_if *iface)
{
    struct wifi_mw320_dev *dev = net_if_get_device(iface)->data;
    dev->iface = iface;

    /* Read MAC address from wlan.h API and set it in Zephyr */
    /* wlan_get_mac_address(dev->mac_addr); */
    
    net_if_set_link_addr(iface, dev->mac_addr, 6, NET_LINK_ETHERNET);
    ethernet_init(iface);
}

static int wifi_mw320_send(const struct device *dev, struct net_pkt *pkt)
{
    /* Convert Zephyr net_pkt to mw320 wlan packet and send */
    return 0;
}

static int wifi_mw320_mgmt_scan(const struct device *dev,
                                struct wifi_scan_params *params,
                                scan_result_cb_t cb)
{
    /* Call wlan_scan() from wlcmgr and pass results to Zephyr cb */
    return 0;
}

static int wifi_mw320_mgmt_connect(const struct device *dev,
                                   struct wifi_connect_req_params *params)
{
    /* Translate Zephyr connect params to wlan_network_t */
    /* wlan_connect(ssid) */
    return 0;
}

static int wifi_mw320_mgmt_disconnect(const struct device *dev)
{
    /* wlan_disconnect() */
    return 0;
}

static const struct wifi_mgmt_ops wifi_mw320_mgmt_ops = {
    .scan       = wifi_mw320_mgmt_scan,
    .connect    = wifi_mw320_mgmt_connect,
    .disconnect = wifi_mw320_mgmt_disconnect,
};

static const struct net_wifi_mgmt_offload wifi_mw320_api = {
    .wifi_iface.iface_api.init = wifi_mw320_iface_init,
    .wifi_iface.send = wifi_mw320_send,
    .wifi_mgmt_api = &wifi_mw320_mgmt_ops,
};

static int wifi_mw320_init(const struct device *dev)
{
    /* Initialize mw320 wlcmgr (wlan_init, wlan_start) */
    wlan_init_network();
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
