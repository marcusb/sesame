#include "leds.h"

#include <stdbool.h>

#include "FreeRTOS.h"
#include "app_logging.h"
#include "mdev_gpio.h"
#include "mdev_pinmux.h"
#include "queue.h"
#include "task.h"

#define LED1 GPIO_16  // ota blue
#define LED2 GPIO_27  // ota green
#define LED3 GPIO_40  // ota red
#define LED4 GPIO_41  // wifi blue
#define LED5 GPIO_42  // wifi green
#define LED6 GPIO_43  // wifi red

static QueueHandle_t msg_queue;

static void gpio_cb(int pin, void* data) {
    xQueueSendToBackFromISR(msg_queue, &pin, NULL);
}

static void init_gpio_input(mdev_t* gpio_dev, int pin) {
    gpio_drv_setdir(gpio_dev, pin, GPIO_INPUT);
    int res =
        gpio_drv_set_cb(gpio_dev, pin, GPIO_INT_BOTH_EDGES, NULL, gpio_cb);
    if (res != WM_SUCCESS) {
        LogError(("failed to regiser GPIO ISR"));
    }
}

static void init_led(mdev_t* gpio_dev, mdev_t* pinmux_dev, int pin) {
    pinmux_drv_setfunc(pinmux_dev, pin, PINMUX_FUNCTION_0);
    gpio_drv_setdir(gpio_dev, pin, GPIO_OUTPUT);
    gpio_drv_write(gpio_dev, pin, GPIO_IO_HIGH);
}

static void process_queue() {
    int pin;
    if (xQueueReceive(msg_queue, &pin, 800) == pdPASS) {
        LogInfo(("GPIO pin %d activated", pin));
    }
}

void led_task(void* const params) {
    LogInfo(("LED control task running"));

    msg_queue = xQueueCreate(5, sizeof(int));

    mdev_t* gpio_dev = gpio_drv_open("MDEV_GPIO");
    if (gpio_dev == NULL) {
        LogError(("gpio_drv_open failed"));
        goto err;
    }
    mdev_t* pinmux_dev = pinmux_drv_open("MDEV_PINMUX");
    if (pinmux_dev == NULL) {
        LogError(("pinux_drv_open failed\r\n"));
        goto err;
    }
    pinmux_drv_setfunc(pinmux_dev, GPIO_22, GPIO22_GPIO22);
    pinmux_drv_setfunc(pinmux_dev, GPIO_23, GPIO23_GPIO23);
    pinmux_drv_setfunc(pinmux_dev, GPIO_24, GPIO24_GPIO24);
    pinmux_drv_setfunc(pinmux_dev, GPIO_47, GPIO47_GPIO47);

    init_gpio_input(gpio_dev, GPIO_22);
    init_gpio_input(gpio_dev, GPIO_23);
    init_gpio_input(gpio_dev, GPIO_24);
    init_gpio_input(gpio_dev, GPIO_47);

    init_led(gpio_dev, pinmux_dev, LED1);
    init_led(gpio_dev, pinmux_dev, LED2);
    init_led(gpio_dev, pinmux_dev, LED3);
    init_led(gpio_dev, pinmux_dev, LED4);
    init_led(gpio_dev, pinmux_dev, LED5);
    init_led(gpio_dev, pinmux_dev, LED6);
    pinmux_drv_close(pinmux_dev);

    for (;;) {
        gpio_drv_write(gpio_dev, LED1, GPIO_IO_LOW);
        gpio_drv_write(gpio_dev, LED2, GPIO_IO_HIGH);
        gpio_drv_write(gpio_dev, LED4, GPIO_IO_HIGH);
        gpio_drv_write(gpio_dev, LED5, GPIO_IO_LOW);
        process_queue();

        gpio_drv_write(gpio_dev, LED1, GPIO_IO_HIGH);
        gpio_drv_write(gpio_dev, LED2, GPIO_IO_LOW);
        gpio_drv_write(gpio_dev, LED4, GPIO_IO_LOW);
        gpio_drv_write(gpio_dev, LED5, GPIO_IO_HIGH);
        process_queue();

        // char s[GPIO_MaxNo + 2];
        // s[sizeof(s) - 1] = 0;
        // for (int i = 0; i <= GPIO_MaxNo; i++) {
        //     int state;
        //     int err = gpio_drv_read(gpio_dev, i, &state);
        //     if (err == WM_SUCCESS) {
        //         s[i] = state ? 'H' : 'L';
        //     } else {
        //         s[i] = 'x';
        //     }
        // }
        // LogInfo((s));
    }
err:
    vTaskDelete(NULL);
}
