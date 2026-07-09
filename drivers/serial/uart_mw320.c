#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#include <string.h>
#include "fsl_uart.h"
#include "fsl_clock.h"

#include <zephyr/drivers/pinctrl.h>

#define DT_DRV_COMPAT nxp_mw320_uart

struct uart_mw320_config {
    UART_Type *base;
    const struct pinctrl_dev_config *pincfg;
};

struct uart_mw320_data {
};

static int uart_mw320_poll_in(const struct device *dev, unsigned char *c)
{
    const struct uart_mw320_config *config = dev->config;

    if (!(UART_GetStatusFlags(config->base) & kUART_RxDataReadyInterruptFlag)) {
        return -1;
    }

    *c = UART_ReadByte(config->base);
    return 0;
}

static void uart_mw320_poll_out(const struct device *dev, unsigned char c)
{
    const struct uart_mw320_config *config = dev->config;

    while (!(UART_GetStatusFlags(config->base) & kUART_TxDataRequestInterruptFlag)) {
    }
    UART_WriteByte(config->base, c);
}

static int uart_mw320_err_check(const struct device *dev)
{
    return 0;
}

static const struct uart_driver_api uart_mw320_driver_api = {
    .poll_in = uart_mw320_poll_in,
    .poll_out = uart_mw320_poll_out,
    .err_check = uart_mw320_err_check,
};

static int uart_mw320_init(const struct device *dev)
{
    const struct uart_mw320_config *config = dev->config;
    uart_config_t uart_cfg;
    int err;

    err = pinctrl_apply_state(config->pincfg, PINCTRL_STATE_DEFAULT);
    if (err < 0) {
        return err;
    }

    CLOCK_EnableClock(kCLOCK_Uart0);
    CLOCK_SetUartClkDiv(kCLOCK_DivUartFast, 2U, 1U);
    CLOCK_AttachClk(kSYS_CLK_to_FAST_UART0);
    uint32_t uart_freq = CLOCK_GetUartClkFreq(0);
    UART_GetDefaultConfig(&uart_cfg);
    uart_cfg.fifoConfig.resetTxFifo = true;
    uart_cfg.fifoConfig.resetRxFifo = true;
    uart_cfg.enable = true;
    uart_cfg.enableHighSpeed = false;
    UART_Init(config->base, &uart_cfg, uart_freq);

    return 0;
}

#define UART_MW320_INIT(n)                                              \
    PINCTRL_DT_INST_DEFINE(n);                                          \
    static const struct uart_mw320_config uart_mw320_config_##n = {     \
        .base = (UART_Type *)DT_INST_REG_ADDR(n),                       \
        .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                    \
    };                                                                  \
    static struct uart_mw320_data uart_mw320_data_##n;                  \
                                                                        \
    DEVICE_DT_INST_DEFINE(n, uart_mw320_init, NULL,                     \
                          &uart_mw320_data_##n,                         \
                          &uart_mw320_config_##n,                       \
                          PRE_KERNEL_1,                                 \
                          CONFIG_SERIAL_INIT_PRIORITY,                  \
                          &uart_mw320_driver_api);

DT_INST_FOREACH_STATUS_OKAY(UART_MW320_INIT)
