#include "gpio.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "queue.h"

// mw320
#include "fsl_gpio.h"

// application
#include "controller.h"
#include "pin_mux.h"

typedef struct {
    int gpio;
    ctrl_msg_type_t msg_type;
} button_def_t;

static const button_def_t buttons[] = {
    {BOARD_SW_WIFI_PIN, CTRL_MSG_WIFI_BUTTON},
    {BOARD_SW_OTA_PIN, CTRL_MSG_OTA_BUTTON},
    {-1}};

void GPIO_IRQHandler(void) {
    int pin;
    for (int i = 0; (pin = buttons[i].gpio) != -1; i++) {
        uint32_t port = GPIO_PORT(pin);
        uint32_t mask = 1UL << GPIO_PORT_PIN(pin);
        if (GPIO_PortGetInterruptFlags(GPIO, port) & mask) {
            GPIO_PortClearInterruptFlags(GPIO, port, mask);
            ctrl_msg_t msg = {buttons[i].msg_type};
            xQueueSendToBackFromISR(ctrl_queue, &msg, NULL);
        }
    }
    SDK_ISR_EXIT_BARRIER;
}
void gpio_set(int pin) {
    GPIO_PortSet(GPIO, GPIO_PORT(pin), 1UL << GPIO_PORT_PIN(pin));
}

void gpio_clear(int pin) {
    GPIO_PortClear(GPIO, GPIO_PORT(pin), 1UL << GPIO_PORT_PIN(pin));
}

static void init_gpio_input(int pin) {
    gpio_pin_config_t sw_config = {
        kGPIO_DigitalInput,
        0,
    };
    GPIO_PinSetInterruptConfig(GPIO, pin, kGPIO_InterruptFallingEdge);
    GPIO_PortEnableInterrupts(GPIO, GPIO_PORT(pin), 1UL << GPIO_PORT_PIN(pin));
    GPIO_PinInit(GPIO, pin, &sw_config);
}

static void gpio_set_output(int pin) {
    gpio_pin_config_t out_config = {
        kGPIO_DigitalOutput,
        0,
    };
    GPIO_PinInit(GPIO, pin, &out_config);
}

void init_gpio() {
    EnableIRQ(GPIO_IRQn);
    // lower interrupt priority so we can call FreeRTOS syscalls from ISR
    NVIC_SetPriority(GPIO_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);

    gpio_set_output(0);
    gpio_clear(0);

    // set_PIC16LF15_SIGNAL_LOW
    // GPIO46 set to RUN (0)
    gpio_set_output(46);
    gpio_clear(46);

    gpio_set_output(48);
    gpio_set(48);

    // set_PIC16LF15_RESET_RELEASE
    // GPIO pin 1 set to RESET (1)
    gpio_set_output(1);
    gpio_clear(1);

    gpio_set_output(49);
    gpio_set(49);

    int pin;
    for (int i = 0; (pin = buttons[i].gpio) != -1; i++) {
        init_gpio_input(pin);
    }

    gpio_set_output(BOARD_LED_OTA_BLUE_PIN);
    gpio_set_output(BOARD_LED_OTA_GREEN_PIN);
    gpio_set_output(BOARD_LED_OTA_RED_PIN);
    gpio_set_output(BOARD_LED_WIFI_BLUE_PIN);
    gpio_set_output(BOARD_LED_WIFI_GREEN_PIN);
    gpio_set_output(BOARD_LED_WIFI_RED_PIN);
    gpio_clear(BOARD_LED_OTA_BLUE_PIN);
    gpio_clear(BOARD_LED_OTA_RED_PIN);
    gpio_set(BOARD_LED_OTA_GREEN_PIN);
    gpio_clear(BOARD_LED_WIFI_BLUE_PIN);
    gpio_clear(BOARD_LED_WIFI_RED_PIN);
    gpio_set(BOARD_LED_WIFI_GREEN_PIN);
}
