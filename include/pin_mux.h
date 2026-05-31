#pragma once

#include <zephyr/drivers/gpio.h>

/* UART pins */
#define BOARD_UART0_TX_PIN 2
#define BOARD_UART0_TX_PIN_FUNCTION_ID PINMUX_GPIO2_UART0_TXD

#define BOARD_UART0_RX_PIN 3
#define BOARD_UART0_RX_PIN_FUNCTION_ID PINMUX_GPIO3_UART0_RXD

#define BOARD_UART1_TX_PIN 44
#define BOARD_UART1_TX_PIN_FUNCTION_ID PINMUX_GPIO44_UART1_TXD

#define BOARD_UART1_RX_PIN 45
#define BOARD_UART1_RX_PIN_FUNCTION_ID PINMUX_GPIO45_UART1_RXD

/* Crystal pins */
#define BOARD_XTAL32K_IN_PIN 25
#define BOARD_XTAL32K_IN_PIN_FUNCTION_ID PINMUX_GPIO25_XTAL32K_IN

#define BOARD_XTAL32K_OUT_PIN 26
#define BOARD_XTAL32K_OUT_PIN_FUNCTION_ID PINMUX_GPIO26_XTAL32K_OUT

/* LED gpio_dt_spec */
static const struct gpio_dt_spec led_ota_blue   = GPIO_DT_SPEC_GET(DT_NODELABEL(led_ota_blue), gpios);
static const struct gpio_dt_spec led_ota_green  = GPIO_DT_SPEC_GET(DT_NODELABEL(led_ota_green), gpios);
static const struct gpio_dt_spec led_ota_red    = GPIO_DT_SPEC_GET(DT_NODELABEL(led_ota_red), gpios);
static const struct gpio_dt_spec led_wifi_blue  = GPIO_DT_SPEC_GET(DT_NODELABEL(led_wifi_blue), gpios);
static const struct gpio_dt_spec led_wifi_green = GPIO_DT_SPEC_GET(DT_NODELABEL(led_wifi_green), gpios);
static const struct gpio_dt_spec led_wifi_red   = GPIO_DT_SPEC_GET(DT_NODELABEL(led_wifi_red), gpios);

/* Button gpio_dt_spec */
static const struct gpio_dt_spec sw_wifi = GPIO_DT_SPEC_GET(DT_NODELABEL(sw_wifi), gpios);
static const struct gpio_dt_spec sw_ota  = GPIO_DT_SPEC_GET(DT_NODELABEL(sw_ota), gpios);

void board_init_pins(void);
