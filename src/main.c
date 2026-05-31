#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#include "pin_mux.h"

int main(void) {
    if (!(device_is_ready(led_ota_blue.port) &&
          device_is_ready(led_ota_green.port) &&
          device_is_ready(led_ota_red.port))) {
        return -ENODEV;
    }

    /* Configure all LEDs as output, initial low */
    gpio_pin_configure_dt(&led_ota_blue, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_ota_green, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_ota_red, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_wifi_blue, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_wifi_green, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&led_wifi_red, GPIO_OUTPUT_INACTIVE);

    /* Blink OTA Blue ~0.5Hz */
    while (1) {
        gpio_pin_set_dt(&led_ota_blue, 1);
        {
            volatile uint32_t c = 20000000U;
            do {
                c--;
            } while (c != 0);
        }
        gpio_pin_set_dt(&led_ota_blue, 0);
        {
            volatile uint32_t c = 20000000U;
            do {
                c--;
            } while (c != 0);
        }
    }
}
