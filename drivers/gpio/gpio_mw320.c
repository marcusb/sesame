#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/irq.h>
#include "fsl_gpio.h"

#define DT_DRV_COMPAT nxp_mw320_gpio

#include <zephyr/drivers/pinctrl.h>
#include "fsl_clock.h"

struct gpio_mw320_config {
    struct gpio_driver_config common;
    GPIO_Type *base;
    uint32_t port;
    const struct pinctrl_dev_config *pincfg;
};

struct gpio_mw320_data {
    struct gpio_driver_data common;
};

static int gpio_mw320_configure(const struct device *dev,
                                gpio_pin_t pin, gpio_flags_t flags)
{
    const struct gpio_mw320_config *config = dev->config;
    gpio_pin_config_t pin_config = {0};
    uint32_t absolute_pin = (config->port * 32U) + pin;

    if (flags & GPIO_OUTPUT) {
        pin_config.pinDirection = kGPIO_DigitalOutput;
        pin_config.outputLogic = (flags & GPIO_OUTPUT_INIT_HIGH) ? 1 : 0;
    } else if (flags & GPIO_INPUT) {
        pin_config.pinDirection = kGPIO_DigitalInput;
    } else {
        return -ENOTSUP;
    }

    GPIO_PinInit(config->base, absolute_pin, &pin_config);
    return 0;
}

static int gpio_mw320_port_get_raw(const struct device *dev, uint32_t *value)
{
    const struct gpio_mw320_config *config = dev->config;
    *value = config->base->GPLR_REG[config->port];
    return 0;
}

static int gpio_mw320_port_set_masked_raw(const struct device *dev,
                                          uint32_t mask, uint32_t value)
{
    const struct gpio_mw320_config *config = dev->config;
    uint32_t out = config->base->GPLR_REG[config->port];
    out = (out & ~mask) | (value & mask);
    /* There is no direct PortWrite, we can use GPSR_REG and GPCR_REG */
    GPIO_PortSet(config->base, config->port, value & mask);
    GPIO_PortClear(config->base, config->port, (~value) & mask);
    return 0;
}

static int gpio_mw320_port_set_bits_raw(const struct device *dev, uint32_t mask)
{
    const struct gpio_mw320_config *config = dev->config;
    GPIO_PortSet(config->base, config->port, mask);
    return 0;
}

static int gpio_mw320_port_clear_bits_raw(const struct device *dev, uint32_t mask)
{
    const struct gpio_mw320_config *config = dev->config;
    GPIO_PortClear(config->base, config->port, mask);
    return 0;
}

static int gpio_mw320_port_toggle_bits(const struct device *dev, uint32_t mask)
{
    const struct gpio_mw320_config *config = dev->config;
    /* There is no PortToggle in earlier fsl_gpio.h or it takes 3 args */
    /* Let's implement toggle using GPLR_REG */
    uint32_t val = config->base->GPLR_REG[config->port];
    GPIO_PortSet(config->base, config->port, (~val) & mask);
    GPIO_PortClear(config->base, config->port, val & mask);
    return 0;
}

static int gpio_mw320_pin_interrupt_configure(const struct device *dev,
                                              gpio_pin_t pin,
                                              enum gpio_int_mode mode,
                                              enum gpio_int_trig trig)
{
    return -ENOTSUP;
}

static const struct gpio_driver_api gpio_mw320_driver_api = {
    .pin_configure = gpio_mw320_configure,
    .port_get_raw = gpio_mw320_port_get_raw,
    .port_set_masked_raw = gpio_mw320_port_set_masked_raw,
    .port_set_bits_raw = gpio_mw320_port_set_bits_raw,
    .port_clear_bits_raw = gpio_mw320_port_clear_bits_raw,
    .port_toggle_bits = gpio_mw320_port_toggle_bits,
    .pin_interrupt_configure = gpio_mw320_pin_interrupt_configure,
};



static int gpio_mw320_init(const struct device *dev)
{
    const struct gpio_mw320_config *config = dev->config;

    CLOCK_EnableClock(kCLOCK_Gpio);

    if (config->pincfg != NULL) {
        pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
    }

    return 0;
}

#define GPIO_MW320_INIT(n)                                              \
    PINCTRL_DT_INST_DEFINE(n);                                          \
    static const struct gpio_mw320_config gpio_mw320_config_##n = {     \
        .common = {                                                     \
            .port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),        \
        },                                                              \
        .base = (GPIO_Type *)DT_INST_REG_ADDR(n),                       \
        .port = DT_INST_PROP(n, port),                                  \
        .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                    \
    };                                                                  \
    static struct gpio_mw320_data gpio_mw320_data_##n;                  \
                                                                        \
    DEVICE_DT_INST_DEFINE(n, gpio_mw320_init, NULL,                     \
                          &gpio_mw320_data_##n,                         \
                          &gpio_mw320_config_##n,                       \
                          PRE_KERNEL_1,                                 \
                          CONFIG_GPIO_INIT_PRIORITY,                    \
                          &gpio_mw320_driver_api);

DT_INST_FOREACH_STATUS_OKAY(GPIO_MW320_INIT)
