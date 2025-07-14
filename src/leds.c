#include "leds.h"

#include <stdbool.h>

#include "app_logging.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

// application
#include "gpio.h"
#include "pin_mux.h"

static uint16_t ota_led_pattern = 0x0101;
static uint16_t wifi_led_pattern = 0x0000;

inline static void set_led_pattern(uint16_t* pattern, uint8_t x1, uint8_t x2,
                                   uint8_t x3, uint8_t x4) {
    *pattern = x1 | (x2 << 4) | (x3 << 8) | (x4 << 12);
}

inline void set_ota_led_pattern(uint8_t x1, uint8_t x2, uint8_t x3,
                                uint8_t x4) {
    set_led_pattern(&ota_led_pattern, x1, x2, x3, x4);
}

inline void set_wifi_led_pattern(uint8_t x1, uint8_t x2, uint8_t x3,
                                 uint8_t x4) {
    set_led_pattern(&wifi_led_pattern, x1, x2, x3, x4);
}

static void set_led(int pin, bool enable) {
    // LEDs are active low
    if (enable) {
        gpio_clear(pin);
    } else {
        gpio_set(pin);
    }
}

inline static void rotate_pattern(uint16_t* pattern) {
    *pattern = (*pattern >> 4) | (*pattern << 12);
}

void led_task(void* const params) {
    LogInfo(("LED control task running"));

    for (;;) {
        set_led(BOARD_LED_OTA_RED_PIN, ota_led_pattern & LED_RED);
        set_led(BOARD_LED_OTA_GREEN_PIN, ota_led_pattern & LED_GREEN);
        set_led(BOARD_LED_OTA_BLUE_PIN, ota_led_pattern & LED_BLUE);
        rotate_pattern(&ota_led_pattern);

        set_led(BOARD_LED_WIFI_RED_PIN, wifi_led_pattern & LED_RED);
        set_led(BOARD_LED_WIFI_GREEN_PIN, wifi_led_pattern & LED_GREEN);
        set_led(BOARD_LED_WIFI_BLUE_PIN, wifi_led_pattern & LED_BLUE);
        rotate_pattern(&wifi_led_pattern);
        vTaskDelay(pdMS_TO_TICKS(400));
    }
}
