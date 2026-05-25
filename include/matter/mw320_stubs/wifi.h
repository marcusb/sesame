#pragma once
/* Minimal wifi.h stub for QEMU builds — satisfies ConfigurationManagerImpl.h
 * which includes wifi.h for wifi_mac_addr_t, wifi_get_device_mac_addr, and
 * PRINTF (the real wifi.h transitively pulls in fsl_debug_console.h).
 * The real implementation is in mw320_sdk (hardware builds only). */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#ifndef PRINTF
#define PRINTF printf
#endif

typedef struct {
    uint8_t mac[6];
} wifi_mac_addr_t;

static inline int wifi_get_device_mac_addr(wifi_mac_addr_t *mac_addr)
{
    static const uint8_t kQemuMac[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};
    memcpy(mac_addr->mac, kQemuMac, 6);
    return 0;
}
