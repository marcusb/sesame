#include <stdbool.h>

#include "FreeRTOS.h"
#include "mdev_gpio.h"
#include "task.h"

#include "leds.h"


#define LED1 16  // ota blue
#define LED2 27  // ota green
#define LED3 40  // ota red
#define LED4 41  // wifi blue
#define LED5 42  // wifi green
#define LED6 43  // wifi red


extern void ledTask(void *const params) {
    configPRINT("LED control task running\r\n");

    gpio_drv_init();
    mdev_t* gpio_dev = gpio_drv_open("MDEV_GPIO");
    if (gpio_dev == NULL) {
        configPRINT("gpio_drv_open failed\r\n");
        return;
    }

    gpio_drv_setdir(gpio_dev, LED1, GPIO_OUTPUT);
    gpio_drv_setdir(gpio_dev, LED2, GPIO_OUTPUT);
    gpio_drv_setdir(gpio_dev, LED3, GPIO_OUTPUT);
    gpio_drv_setdir(gpio_dev, LED4, GPIO_OUTPUT);
    gpio_drv_setdir(gpio_dev, LED5, GPIO_OUTPUT);
    gpio_drv_setdir(gpio_dev, LED6, GPIO_OUTPUT);

    gpio_drv_write(gpio_dev, LED1, GPIO_IO_HIGH);
    gpio_drv_write(gpio_dev, LED2, GPIO_IO_HIGH);
    gpio_drv_write(gpio_dev, LED3, GPIO_IO_HIGH);
    gpio_drv_write(gpio_dev, LED4, GPIO_IO_HIGH);
    gpio_drv_write(gpio_dev, LED5, GPIO_IO_HIGH);
    gpio_drv_write(gpio_dev, LED6, GPIO_IO_HIGH);

    while (true) {
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
}
