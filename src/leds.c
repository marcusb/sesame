#include "leds.h"

#include <stdbool.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(leds, LOG_LEVEL_DBG);

#define LED_STACK_SIZE 1024
#define LED_PRIORITY 7

K_THREAD_STACK_DEFINE(led_stack_area, LED_STACK_SIZE);
static struct k_thread led_thread_data;

static uint16_t ota_led_pattern = 0x0101;
static uint16_t wifi_led_pattern = 0x0000;

static const struct gpio_dt_spec ota_red =
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_ota_red), gpios);
static const struct gpio_dt_spec ota_green =
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_ota_green), gpios);
static const struct gpio_dt_spec ota_blue =
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_ota_blue), gpios);

static const struct gpio_dt_spec wifi_red =
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_wifi_red), gpios);
static const struct gpio_dt_spec wifi_green =
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_wifi_green), gpios);
static const struct gpio_dt_spec wifi_blue =
    GPIO_DT_SPEC_GET(DT_NODELABEL(led_wifi_blue), gpios);

static inline void set_led_pattern(uint16_t* pattern, uint8_t x1, uint8_t x2,
                                   uint8_t x3, uint8_t x4) {
    *pattern = x1 | (x2 << 4) | (x3 << 8) | (x4 << 12);
}

void set_ota_led_pattern(uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4) {
    set_led_pattern(&ota_led_pattern, x1, x2, x3, x4);
}

void set_wifi_led_pattern(uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4) {
    set_led_pattern(&wifi_led_pattern, x1, x2, x3, x4);
}

static inline void rotate_pattern(uint16_t* pattern) {
    *pattern = (*pattern >> 4) | (*pattern << 12);
}

static void led_thread(void* p1, void* p2, void* p3) {
    LOG_INF("LED control task running");

    gpio_pin_configure_dt(&ota_red, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&ota_green, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&ota_blue, GPIO_OUTPUT_INACTIVE);

    gpio_pin_configure_dt(&wifi_red, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&wifi_green, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&wifi_blue, GPIO_OUTPUT_INACTIVE);

    for (;;) {
        gpio_pin_set_dt(&ota_red, ota_led_pattern & LED_RED);
        gpio_pin_set_dt(&ota_green, ota_led_pattern & LED_GREEN);
        gpio_pin_set_dt(&ota_blue, ota_led_pattern & LED_BLUE);
        rotate_pattern(&ota_led_pattern);

        gpio_pin_set_dt(&wifi_red, wifi_led_pattern & LED_RED);
        gpio_pin_set_dt(&wifi_green, wifi_led_pattern & LED_GREEN);
        gpio_pin_set_dt(&wifi_blue, wifi_led_pattern & LED_BLUE);
        rotate_pattern(&wifi_led_pattern);

        k_msleep(400);
    }
}

void leds_init(void) {
    k_thread_create(&led_thread_data, led_stack_area,
                    K_THREAD_STACK_SIZEOF(led_stack_area), led_thread, NULL,
                    NULL, NULL, LED_PRIORITY, 0, K_NO_WAIT);
}
