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
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    uart_irq_config_func_t irq_config_func;
#endif
};

struct uart_mw320_data {
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    uart_irq_callback_user_data_t cb;
    void *cb_data;
#endif
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

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int uart_mw320_fifo_fill(const struct device *dev, const uint8_t *tx_data, int size)
{
    const struct uart_mw320_config *config = dev->config;
    int num_tx = 0;

    while (size - num_tx > 0 && (UART_GetStatusFlags(config->base) & kUART_TxDataRequestInterruptFlag)) {
        UART_WriteByte(config->base, tx_data[num_tx++]);
    }
    return num_tx;
}

static int uart_mw320_fifo_read(const struct device *dev, uint8_t *rx_data, const int size)
{
    const struct uart_mw320_config *config = dev->config;
    int num_rx = 0;

    while (size - num_rx > 0 && (UART_GetStatusFlags(config->base) & kUART_RxDataReadyInterruptFlag)) {
        rx_data[num_rx++] = UART_ReadByte(config->base);
    }
    return num_rx;
}

static void uart_mw320_irq_tx_enable(const struct device *dev)
{
    const struct uart_mw320_config *config = dev->config;
    UART_EnableInterrupts(config->base, kUART_TxDataRequestInterruptEnable);
}

static void uart_mw320_irq_tx_disable(const struct device *dev)
{
    const struct uart_mw320_config *config = dev->config;
    UART_DisableInterrupts(config->base, kUART_TxDataRequestInterruptEnable);
}

static int uart_mw320_irq_tx_ready(const struct device *dev)
{
    const struct uart_mw320_config *config = dev->config;
    return (UART_GetStatusFlags(config->base) & kUART_TxDataRequestInterruptFlag) != 0;
}

static void uart_mw320_irq_rx_enable(const struct device *dev)
{
    const struct uart_mw320_config *config = dev->config;
    UART_EnableInterrupts(config->base, kUART_RxDataReadyInterruptEnable);
}

static void uart_mw320_irq_rx_disable(const struct device *dev)
{
    const struct uart_mw320_config *config = dev->config;
    UART_DisableInterrupts(config->base, kUART_RxDataReadyInterruptEnable);
}

static int uart_mw320_irq_tx_complete(const struct device *dev)
{
    const struct uart_mw320_config *config = dev->config;
    return (UART_GetStatusFlags(config->base) & kUART_TxEmptyInterruptFlag) != 0;
}

static int uart_mw320_irq_rx_ready(const struct device *dev)
{
    const struct uart_mw320_config *config = dev->config;
    return (UART_GetStatusFlags(config->base) & kUART_RxDataReadyInterruptFlag) != 0;
}

static void uart_mw320_irq_err_enable(const struct device *dev)
{
}

static void uart_mw320_irq_err_disable(const struct device *dev)
{
}

static int uart_mw320_irq_is_pending(const struct device *dev)
{
    return uart_mw320_irq_tx_ready(dev) || uart_mw320_irq_rx_ready(dev);
}

static void uart_mw320_irq_update(const struct device *dev)
{
    
}

static void uart_mw320_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb, void *cb_data)
{
    struct uart_mw320_data *data = dev->data;
    data->cb = cb;
    data->cb_data = cb_data;
}

static void uart_mw320_isr(const struct device *dev)
{
    struct uart_mw320_data *data = dev->data;
    if (data->cb) {
        data->cb(dev, data->cb_data);
    } else {
        const struct uart_mw320_config *config = dev->config;
        UART_DisableInterrupts(config->base, kUART_RxDataReadyInterruptEnable | kUART_TxDataRequestInterruptEnable);
    }
}
#endif

static int uart_mw320_configure(const struct device *dev, const struct uart_config *cfg)
{
    const struct uart_mw320_config *config = dev->config;
    uart_config_t uart_cfg;
    uint32_t uart_freq = 0;

    if (config->base == UART0) {
        uart_freq = CLOCK_GetUartClkFreq(0);
    } else if (config->base == UART1) {
        uart_freq = CLOCK_GetUartClkFreq(1);
    } else if (config->base == UART2) {
        uart_freq = CLOCK_GetUartClkFreq(2);
    }

    UART_GetDefaultConfig(&uart_cfg);
    uart_cfg.baudRate_Bps = cfg->baudrate;

    if (cfg->parity == UART_CFG_PARITY_NONE) {
        uart_cfg.parityMode = kUART_ParityDisabled;
    } else if (cfg->parity == UART_CFG_PARITY_ODD) {
        uart_cfg.parityMode = kUART_ParityOdd;
    } else if (cfg->parity == UART_CFG_PARITY_EVEN) {
        uart_cfg.parityMode = kUART_ParityEven;
    } else {
        return -ENOTSUP;
    }

    if (cfg->stop_bits == UART_CFG_STOP_BITS_1) {
        uart_cfg.stopBitCount = kUART_Stopbits1;
    } else if (cfg->stop_bits == UART_CFG_STOP_BITS_2) {
        uart_cfg.stopBitCount = kUART_Stopbits2;
    } else {
        return -ENOTSUP;
    }

    if (cfg->data_bits == UART_CFG_DATA_BITS_5) {
        uart_cfg.dataBitCount = kUART_Databits5;
    } else if (cfg->data_bits == UART_CFG_DATA_BITS_6) {
        uart_cfg.dataBitCount = kUART_Databits6;
    } else if (cfg->data_bits == UART_CFG_DATA_BITS_7) {
        uart_cfg.dataBitCount = kUART_Databits7;
    } else if (cfg->data_bits == UART_CFG_DATA_BITS_8) {
        uart_cfg.dataBitCount = kUART_Databits8;
    } else {
        return -ENOTSUP;
    }

    if (cfg->flow_ctrl != UART_CFG_FLOW_CTRL_NONE) {
        return -ENOTSUP;
    }

    uart_cfg.fifoConfig.resetTxFifo = false;
    uart_cfg.fifoConfig.resetRxFifo = false;
    uart_cfg.enable = true;
    uart_cfg.enableHighSpeed = false;

    if (UART_Init(config->base, &uart_cfg, uart_freq) != kStatus_Success) {
        return -EINVAL;
    }

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    struct uart_mw320_data *data = dev->data;
    if (data->cb) {
        UART_EnableInterrupts(config->base, kUART_RxDataReadyInterruptEnable);
    }
#endif
    return 0;
}

static int uart_mw320_config_get(const struct device *dev, struct uart_config *cfg)
{
    return -ENOTSUP;
}

static const struct uart_driver_api uart_mw320_driver_api = {
    .poll_in = uart_mw320_poll_in,
    .poll_out = uart_mw320_poll_out,
    .err_check = uart_mw320_err_check,
    .configure = uart_mw320_configure,
    .config_get = uart_mw320_config_get,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    .fifo_fill = uart_mw320_fifo_fill,
    .fifo_read = uart_mw320_fifo_read,
    .irq_tx_enable = uart_mw320_irq_tx_enable,
    .irq_tx_disable = uart_mw320_irq_tx_disable,
    .irq_tx_ready = uart_mw320_irq_tx_ready,
    .irq_rx_enable = uart_mw320_irq_rx_enable,
    .irq_rx_disable = uart_mw320_irq_rx_disable,
    .irq_tx_complete = uart_mw320_irq_tx_complete,
    .irq_rx_ready = uart_mw320_irq_rx_ready,
    .irq_err_enable = uart_mw320_irq_err_enable,
    .irq_err_disable = uart_mw320_irq_err_disable,
    .irq_is_pending = uart_mw320_irq_is_pending,
    .irq_update = uart_mw320_irq_update,
    .irq_callback_set = uart_mw320_irq_callback_set,
#endif
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

    uint32_t uart_freq = 0;
    if (config->base == UART0) { // UART0
        CLOCK_EnableClock(kCLOCK_Uart0);
        CLOCK_SetUartClkDiv(kCLOCK_DivUartFast, 2U, 1U);
        CLOCK_AttachClk(kSYS_CLK_to_FAST_UART0);
        uart_freq = CLOCK_GetUartClkFreq(0);
    } else if (config->base == UART1) { // UART1
        CLOCK_EnableClock(kCLOCK_Uart1);
        CLOCK_AttachClk(kSYS_CLK_to_SLOW_UART1);
        for (volatile int i = 0; i < 50000; i++) {} // delay for clock stabilization
        uart_freq = CLOCK_GetUartClkFreq(1);
    }

    UART_GetDefaultConfig(&uart_cfg);
    uart_cfg.fifoConfig.resetTxFifo = true;
    uart_cfg.fifoConfig.resetRxFifo = true;
    uart_cfg.enable = true;
    uart_cfg.enableHighSpeed = false;
    UART_Init(config->base, &uart_cfg, uart_freq);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    config->irq_config_func(dev);
#endif

    return 0;
}

#define UART_MW320_INIT(n)                                              \
    PINCTRL_DT_INST_DEFINE(n);                                          \
    static void uart_mw320_irq_config_##n(const struct device *dev);    \
    static const struct uart_mw320_config uart_mw320_config_##n = {     \
        .base = (UART_Type *)DT_INST_REG_ADDR(n),                       \
        .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(n),                    \
        IF_ENABLED(CONFIG_UART_INTERRUPT_DRIVEN, (.irq_config_func = uart_mw320_irq_config_##n,)) \
    };                                                                  \
    static struct uart_mw320_data uart_mw320_data_##n;                  \
                                                                        \
    DEVICE_DT_INST_DEFINE(n, uart_mw320_init, NULL,                     \
                          &uart_mw320_data_##n,                         \
                          &uart_mw320_config_##n,                       \
                          PRE_KERNEL_1,                                 \
                          CONFIG_SERIAL_INIT_PRIORITY,                  \
                          &uart_mw320_driver_api);                      \
                                                                        \
    static void uart_mw320_irq_config_##n(const struct device *dev)     \
    {                                                                   \
        IF_ENABLED(CONFIG_UART_INTERRUPT_DRIVEN, (                      \
            IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),      \
                        uart_mw320_isr, DEVICE_DT_INST_GET(n), 0);      \
            irq_enable(DT_INST_IRQN(n));                                \
        ))                                                              \
    }

DT_INST_FOREACH_STATUS_OKAY(UART_MW320_INIT)
