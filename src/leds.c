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

static void init_led(mdev_t* gpio_dev, mdev_t* pinmux_dev, int pin) {
    pinmux_drv_setfunc(pinmux_dev, pin, PINMUX_FUNCTION_0);
    gpio_drv_setdir(gpio_dev, pin, GPIO_OUTPUT);
    gpio_drv_write(gpio_dev, pin, GPIO_IO_HIGH);
}

void led_task(void* const params) {
    LogInfo(("LED control task running"));

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
        vTaskDelay(800);

        gpio_drv_write(gpio_dev, LED1, GPIO_IO_HIGH);
        gpio_drv_write(gpio_dev, LED2, GPIO_IO_LOW);
        gpio_drv_write(gpio_dev, LED4, GPIO_IO_LOW);
        gpio_drv_write(gpio_dev, LED5, GPIO_IO_HIGH);
        vTaskDelay(800);
    }
err:
    vTaskDelete(NULL);
}
