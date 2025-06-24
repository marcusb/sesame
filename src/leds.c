#include "leds.h"

#include "app_logging.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

// mw320
#include "fsl_gpio.h"

// application
#include "board.h"
#include "gpio.h"
#include "pin_mux.h"

void gpio_set(int pin) {
    GPIO_PortSet(GPIO, GPIO_PORT(pin), 1UL << GPIO_PORT_PIN(pin));
}

void gpio_clear(int pin) {
    GPIO_PortClear(GPIO, GPIO_PORT(pin), 1UL << GPIO_PORT_PIN(pin));
}

void gpio_set_output(int pin) {
    gpio_pin_config_t out_config = {
        kGPIO_DigitalOutput,
        0,
    };
    GPIO_PinInit(GPIO, pin, &out_config);
}

static void init_led(int pin) {
    gpio_set_output(pin);
    gpio_set(pin);
}

void led_task(void* const params) {
    LogInfo(("LED control task running"));

    init_led(BOARD_LED_OTA_BLUE_PIN);
    init_led(BOARD_LED_OTA_GREEN_PIN);
    init_led(BOARD_LED_OTA_RED_PIN);
    init_led(BOARD_LED_WIFI_BLUE_PIN);
    init_led(BOARD_LED_WIFI_GREEN_PIN);
    init_led(BOARD_LED_WIFI_RED_PIN);

    for (;;) {
        gpio_clear(BOARD_LED_OTA_BLUE_PIN);
        gpio_set(BOARD_LED_OTA_GREEN_PIN);
        gpio_set(BOARD_LED_WIFI_BLUE_PIN);
        gpio_clear(BOARD_LED_WIFI_GREEN_PIN);
        vTaskDelay(800);

        gpio_set(BOARD_LED_OTA_BLUE_PIN);
        gpio_clear(BOARD_LED_OTA_GREEN_PIN);
        gpio_clear(BOARD_LED_WIFI_BLUE_PIN);
        gpio_set(BOARD_LED_WIFI_GREEN_PIN);
        vTaskDelay(800);
    }
}
