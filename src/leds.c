#include "leds.h"

#include <stdbool.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/wifi_mgmt.h>

LOG_MODULE_REGISTER(leds, LOG_LEVEL_DBG);

#define LED_STACK_SIZE 1024
#define LED_PRIORITY 7

K_THREAD_STACK_DEFINE(led_stack_area, LED_STACK_SIZE);
static struct k_thread led_thread_data;

static uint16_t ota_led_pattern = 0x0101;
static uint16_t wifi_led_pattern = 0x0000;

static struct net_mgmt_event_callback l4_mgmt_cb;
static struct net_mgmt_event_callback wifi_mgmt_cb;

#if DT_NODE_EXISTS(DT_NODELABEL(led_wifi_green))

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

static void led_event_handler(struct net_mgmt_event_callback* cb,
                              uint64_t mgmt_event, struct net_if* iface) {
    switch (mgmt_event) {
        case NET_EVENT_L4_CONNECTED:
            set_wifi_led_pattern(LED_GREEN, LED_GREEN, LED_GREEN, LED_GREEN);
            break;
        case NET_EVENT_L4_DISCONNECTED:
            set_wifi_led_pattern(LED_GREEN, LED_OFF, LED_GREEN, LED_OFF);
            break;
        case NET_EVENT_WIFI_AP_ENABLE_RESULT:
            set_wifi_led_pattern(LED_BLUE, LED_OFF, LED_BLUE, LED_OFF);
            break;
        case NET_EVENT_WIFI_AP_DISABLE_RESULT:
            set_wifi_led_pattern(LED_OFF, LED_OFF, LED_OFF, LED_OFF);
            break;
        default:
            break;
    }
}

void leds_init(void) {
    k_thread_create(&led_thread_data, led_stack_area,
                    K_THREAD_STACK_SIZEOF(led_stack_area), led_thread, NULL,
                    NULL, NULL, LED_PRIORITY, 0, K_NO_WAIT);

    net_mgmt_init_event_callback(
        &l4_mgmt_cb, led_event_handler,
        NET_EVENT_L4_CONNECTED | NET_EVENT_L4_DISCONNECTED);
    net_mgmt_add_event_callback(&l4_mgmt_cb);

    net_mgmt_init_event_callback(
        &wifi_mgmt_cb, led_event_handler,
        NET_EVENT_WIFI_AP_ENABLE_RESULT | NET_EVENT_WIFI_AP_DISABLE_RESULT);
    net_mgmt_add_event_callback(&wifi_mgmt_cb);
}

#else

void set_ota_led_pattern(uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4) {}
void set_wifi_led_pattern(uint8_t x1, uint8_t x2, uint8_t x3, uint8_t x4) {}
void leds_init(void) {}

#endif
