#include <stdint.h>
#include <stdio.h>

#include "leds.h"
#include "ota.h"

// OTA Stubs
ota_status_t ota_status = OTA_STATUS_NONE;
void check_ota_test_image() {}
int ota_promote_image() { return 0; }

// LED Stubs
void set_wifi_led_pattern(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4) {}
void set_ota_led_pattern(uint8_t p1, uint8_t p2, uint8_t p3, uint8_t p4) {}

// Network Manager Stubs
void notify_dhcp_configured() {}
void notify_ipv6_addr_change() {}
void wifi_deinit() {}

void WDT_Refresh(void* base) {}
void board_refresh_watchdog(void) {}

const char* pcApplicationHostnameHook() { return "sesame-qemu"; }

/* WiFi Stubs */
typedef struct {
    uint8_t mac[6];
} wifi_mac_addr_t;

#include "FreeRTOS_IP.h"
#include "NetworkInterface.h"

int wifi_get_device_mac_addr(wifi_mac_addr_t* mac_addr) {
    /* Retrieve the MAC address from the first FreeRTOS endpoint.
     * In QEMU, this is the Ethernet interface initialized in qemu_main.c.
     * We use a weak reference so tests that don't link qemu_main.c still link.
     */
    extern NetworkInterface_t eth_iface __attribute__((weak));
    if (&eth_iface) {
        NetworkEndPoint_t* ep = FreeRTOS_FirstEndPoint(&eth_iface);
        if (ep) {
            memcpy(mac_addr->mac, ep->xMACAddress.ucBytes, 6);
            return 0; /* WM_SUCCESS */
        }
    }

    /* Fallback if network not yet initialized or eth_iface not linked */
    mac_addr->mac[0] = 0x00;
    mac_addr->mac[1] = 0x11;
    mac_addr->mac[2] = 0x22;
    mac_addr->mac[3] = 0x33;
    mac_addr->mac[4] = 0x44;
    mac_addr->mac[5] = 0x55;
    return 0; /* WM_SUCCESS */
}
