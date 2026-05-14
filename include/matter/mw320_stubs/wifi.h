#pragma once
/* Minimal wifi.h stub for QEMU builds — satisfies ConfigurationManagerImpl.h
 * which includes wifi.h for wifi_mac_addr_t, wifi_get_device_mac_addr, and
 * PRINTF (the real wifi.h transitively pulls in fsl_debug_console.h).
 * The real implementation is in mw320_sdk (hardware builds only). */

#include <stdint.h>
#include <stdio.h>

#ifndef PRINTF
#define PRINTF printf
#endif

typedef struct {
    uint8_t mac[6];
} wifi_mac_addr_t;

static inline int wifi_get_device_mac_addr(wifi_mac_addr_t *mac_addr)
{
    (void)mac_addr;
    return -1;
}
