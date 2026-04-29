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
