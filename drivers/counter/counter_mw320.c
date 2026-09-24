/*
 * Copyright 2026 Marcus Better
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_mw320_rtc

#include <zephyr/device.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "fsl_clock.h"
#include "fsl_rtc.h"

LOG_MODULE_REGISTER(counter_mw320, CONFIG_COUNTER_LOG_LEVEL);

struct counter_mw320_config {
    struct counter_config_info info;
    RTC_Type* base;
    rtc_clock_div_t clk_div;
    void (*irq_config_func)(const struct device* dev);
};

struct counter_mw320_data {
    counter_alarm_callback_t alarm_callback;
    void* alarm_user_data;
    counter_top_callback_t top_callback;
    void* top_user_data;
    uint32_t overflow_count;
};

static int counter_mw320_start(const struct device* dev) {
    const struct counter_mw320_config* config = dev->config;

    RTC_StartTimer(config->base);
    RTC_EnableInterrupts(config->base,
                         (uint32_t)kRTC_TimeOverflowInterruptEnable);
    return 0;
}

static int counter_mw320_stop(const struct device* dev) {
    const struct counter_mw320_config* config = dev->config;

    RTC_DisableInterrupts(config->base, (uint32_t)kRTC_AllInterruptsEnable);
    RTC_StopTimer(config->base);
    return 0;
}

static int counter_mw320_get_value(const struct device* dev, uint32_t* ticks) {
    const struct counter_mw320_config* config = dev->config;

    *ticks = RTC_GetCounter(config->base);
    return 0;
}

static int counter_mw320_set_channel_alarm(
    const struct device* dev, uint8_t chan_id,
    const struct counter_alarm_cfg* alarm_cfg) {
    const struct counter_mw320_config* config = dev->config;
    struct counter_mw320_data* data = dev->data;

    if (chan_id != 0U) {
        return -EINVAL;
    }

    if (data->alarm_callback != NULL) {
        return -EBUSY;
    }

    uint32_t current = RTC_GetCounter(config->base);
    uint32_t alarm_ticks = alarm_cfg->ticks;

    if ((alarm_cfg->flags & COUNTER_ALARM_CFG_ABSOLUTE) == 0U) {
        alarm_ticks += current;
    }

    if (alarm_ticks <= current) {
        if ((alarm_cfg->flags & COUNTER_ALARM_CFG_EXPIRE_WHEN_LATE) != 0U) {
            alarm_ticks = current + 1U;
        } else {
            return -ETIME;
        }
    }

    data->alarm_callback = alarm_cfg->callback;
    data->alarm_user_data = alarm_cfg->user_data;

    RTC_SetAlarm(config->base, alarm_ticks);
    RTC_EnableInterrupts(config->base, (uint32_t)kRTC_AlarmInterruptEnable);

    return 0;
}

static int counter_mw320_cancel_alarm(const struct device* dev,
                                      uint8_t chan_id) {
    const struct counter_mw320_config* config = dev->config;
    struct counter_mw320_data* data = dev->data;

    if (chan_id != 0U) {
        return -EINVAL;
    }

    RTC_DisableInterrupts(config->base, (uint32_t)kRTC_AlarmInterruptEnable);
    data->alarm_callback = NULL;

    return 0;
}

static int counter_mw320_set_top_value(const struct device* dev,
                                       const struct counter_top_cfg* cfg) {
    const struct counter_mw320_config* config = dev->config;
    struct counter_mw320_data* data = dev->data;

    if (cfg->ticks == 0U) {
        return -EINVAL;
    }

    config->base->CNT_UPP_VAL_REG = cfg->ticks;
    data->top_callback = cfg->callback;
    data->top_user_data = cfg->user_data;

    if ((cfg->flags & COUNTER_TOP_CFG_DONT_RESET) == 0U) {
        RTC_ResetTimer(config->base);
    }

    return 0;
}

static uint32_t counter_mw320_get_pending_int(const struct device* dev) {
    const struct counter_mw320_config* config = dev->config;

    return (RTC_GetStatusFlags(config->base) & (uint32_t)kRTC_AlarmFlag) ? 1U
                                                                         : 0U;
}

static uint32_t counter_mw320_get_top_value(const struct device* dev) {
    const struct counter_mw320_config* config = dev->config;

    return config->base->CNT_UPP_VAL_REG;
}

static uint32_t counter_mw320_get_freq(const struct device* dev) {
    const struct counter_mw320_config* config = dev->config;

    return config->info.freq;
}

static int counter_mw320_reset(const struct device* dev) {
    const struct counter_mw320_config* config = dev->config;

    RTC_ResetTimer(config->base);
    return 0;
}

static void counter_mw320_isr(const struct device* dev) {
    const struct counter_mw320_config* config = dev->config;
    struct counter_mw320_data* data = dev->data;
    uint32_t status = RTC_GetStatusFlags(config->base);

    if ((status & (uint32_t)kRTC_AlarmFlag) != 0U) {
        RTC_DisableInterrupts(config->base,
                              (uint32_t)kRTC_AlarmInterruptEnable);
        RTC_ClearStatusFlags(config->base, (uint32_t)kRTC_AlarmFlag);

        if (data->alarm_callback != NULL) {
            counter_alarm_callback_t cb = data->alarm_callback;
            void* user_data = data->alarm_user_data;

            data->alarm_callback = NULL;
            cb(dev, 0, RTC_GetCounter(config->base), user_data);
        }
    }

    if ((status & (uint32_t)kRTC_TimeOverflowFlag) != 0U) {
        RTC_ClearStatusFlags(config->base, (uint32_t)kRTC_TimeOverflowFlag);
        data->overflow_count++;

        if (data->top_callback != NULL) {
            data->top_callback(dev, data->top_user_data);
        }
    }
}

static int counter_mw320_init(const struct device* dev) {
    const struct counter_mw320_config* config = dev->config;

    /* Enable XTAL32K and attach to RTC */
    CLOCK_EnableXtal32K(kCLOCK_Osc32k_Internal);
    CLOCK_AttachClk(kXTAL32K_to_RTC);

    /* Initialize RTC hardware */
    rtc_config_t rtc_cfg;
    RTC_GetDefaultConfig(&rtc_cfg);
    rtc_cfg.clkDiv = config->clk_div;
    rtc_cfg.cntUppVal = config->info.max_top_value;
    rtc_cfg.autoUpdateCntVal = true;
    rtc_cfg.ignoreInRunning = true; /* Don't reset counter if already running */

    RTC_Init(config->base, &rtc_cfg);

    /* Configure and enable IRQ */
    config->irq_config_func(dev);

    /* Start the timer */
    counter_mw320_start(dev);

    return 0;
}

static DEVICE_API(counter, counter_mw320_driver_api) = {
    .start = counter_mw320_start,
    .stop = counter_mw320_stop,
    .get_value = counter_mw320_get_value,
    .set_alarm = counter_mw320_set_channel_alarm,
    .cancel_alarm = counter_mw320_cancel_alarm,
    .set_top_value = counter_mw320_set_top_value,
    .get_pending_int = counter_mw320_get_pending_int,
    .get_top_value = counter_mw320_get_top_value,
    .get_freq = counter_mw320_get_freq,
    .reset = counter_mw320_reset,
};

#define COUNTER_MW320_INIT(n)                                                  \
    static void counter_mw320_irq_config_##n(const struct device* dev);        \
                                                                               \
    static const struct counter_mw320_config counter_mw320_config_##n = {      \
        .info =                                                                \
            {                                                                  \
                .max_top_value = UINT32_MAX,                                   \
                .freq =                                                        \
                    32768U / (1U << DT_INST_PROP_OR(n, clock_divider_exp, 5)), \
                .flags = COUNTER_CONFIG_INFO_COUNT_UP,                         \
                .channels = 1,                                                 \
            },                                                                 \
        .base = (RTC_Type*)DT_INST_REG_ADDR(n),                                \
        .clk_div = (rtc_clock_div_t)DT_INST_PROP_OR(n, clock_divider_exp, 5),  \
        .irq_config_func = counter_mw320_irq_config_##n,                       \
    };                                                                         \
                                                                               \
    static struct counter_mw320_data counter_mw320_data_##n;                   \
                                                                               \
    DEVICE_DT_INST_DEFINE(n, counter_mw320_init, NULL,                         \
                          &counter_mw320_data_##n, &counter_mw320_config_##n,  \
                          POST_KERNEL, CONFIG_COUNTER_INIT_PRIORITY,           \
                          &counter_mw320_driver_api);                          \
                                                                               \
    static void counter_mw320_irq_config_##n(const struct device* dev) {       \
        IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),                 \
                    counter_mw320_isr, DEVICE_DT_INST_GET(n), 0);              \
        irq_enable(DT_INST_IRQN(n));                                           \
    }

DT_INST_FOREACH_STATUS_OKAY(COUNTER_MW320_INIT)
