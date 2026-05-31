#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#include <string.h>
#include "fsl_uart.h"
#include "fsl_clock.h"
#include "fsl_pinmux.h"
#include "88MW320.h"

#define DT_DRV_COMPAT nxp_mw320_uart

struct uart_mw320_config {
    UART_Type *base;
    uint32_t baud_rate;
};

struct uart_mw320_data {
    struct uart_config uart_cfg;
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

extern volatile uint32_t boot_diag;
volatile uint32_t uart_clk_freq __attribute__((section(".retained")));

static int uart_mw320_init(const struct device *dev)
{
    const struct uart_mw320_config *config = dev->config;
    uart_config_t uart_cfg;
    UART_GetDefaultConfig(&uart_cfg);
    uart_cfg.baudRate_Bps = 115200U;
    uart_cfg.enable = true;

    boot_diag = 0x10;
    CLOCK_AttachClk(kSYS_CLK_to_FAST_UART0);
    boot_diag = 0x11;
    CLOCK_SetUartClkDiv(kCLOCK_DivUartFast, 1, 1);
    boot_diag = 0x12;
    uint32_t uart_freq = CLOCK_GetUartClkFreq(0);
    uart_clk_freq = uart_freq;
    boot_diag = 0x13;
    UART_Init(config->base, &uart_cfg, uart_freq);
    boot_diag = 0x14;

    return 0;
}

#define UART_MW320_INIT(n)                                              \
    static const struct uart_mw320_config uart_mw320_config_##n = {     \
        .base = (UART_Type *)DT_INST_REG_ADDR(n),                       \
        .baud_rate = DT_INST_PROP(n, current_speed),                    \
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
