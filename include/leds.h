#pragma once

#include <stdint.h>

#define LED_OFF (0)
#define LED_RED (1 << 0)
#define LED_GREEN (1 << 1)
#define LED_BLUE (1 << 2)

void led_task(void *params);
void set_ota_led_pattern(uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4);
void set_wifi_led_pattern(uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4);
