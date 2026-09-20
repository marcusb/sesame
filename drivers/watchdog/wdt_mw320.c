#include <errno.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>

#include "fsl_clock.h"
#include "fsl_wdt.h"

#define DT_DRV_COMPAT nxp_mw320_wdt

struct wdt_mw320_data {
    wdt_timeout_value_t timeout_val;
    uint32_t clk_div;
    bool configured;
};

static struct wdt_mw320_data s_wdt_data;

static int wdt_mw320_setup(const struct device* dev, uint8_t options) {
    ARG_UNUSED(dev);
    ARG_UNUSED(options);

    wdt_config_t config;
    WDT_GetDefaultConfig(&config);

    if (s_wdt_data.configured) {
        CLOCK_SetClkDiv(kCLOCK_DivWdt, s_wdt_data.clk_div);
        config.timeoutValue = s_wdt_data.timeout_val;
    } else {
        config.timeoutValue = kWDT_TimeoutVal2ToThePowerOf31;
    }

    config.timeoutMode = kWDT_ModeTimeoutReset;
    config.enableWDT = true;

    WDT_Init(WDT, &config);
    return 0;
}

static int wdt_mw320_disable(const struct device* dev) {
    ARG_UNUSED(dev);
    /* Disable watchdog timer immediately */
    WDT->WDT_CR = 0;
    return 0;
}

static int wdt_mw320_install_timeout(const struct device* dev,
                                     const struct wdt_timeout_cfg* cfg) {
    ARG_UNUSED(dev);

    if (cfg->window.min != 0U || cfg->window.max == 0U) {
        return -EINVAL;
    }

    uint32_t apb_freq = CLOCK_GetApbFreq(1U);
    if (apb_freq == 0U) {
        apb_freq = 100000000U;
    }

    /* Target number of base clock cycles for requested timeout in milliseconds
     */
    uint64_t target_cycles =
        ((uint64_t)cfg->window.max * (uint64_t)apb_freq) / 1000ULL;

    /*
     * We need 2^(p + clk_div) >= target_cycles.
     * p is the hardware counter exponent: 16 <= p <= 31
     * (kWDT_TimeoutVal2ToThePowerOf16..31). clk_div is the peripheral divider
     * exponent: wdt_freq = apb_freq >> clk_div.
     */
    uint32_t total_exp = 16;
    while (total_exp < 63 && ((1ULL << total_exp) < target_cycles)) {
        total_exp++;
    }

    uint32_t p = total_exp;
    uint32_t clk_div = 0;
    if (p > 31) {
        clk_div = p - 31;
        p = 31;
        if (clk_div > 0x3F) {
            clk_div = 0x3F;
        }
    }

    s_wdt_data.timeout_val = (wdt_timeout_value_t)(p - 16);
    s_wdt_data.clk_div = clk_div;
    s_wdt_data.configured = true;

    return 0;
}

static int wdt_mw320_feed(const struct device* dev, int channel_id) {
    ARG_UNUSED(dev);
    ARG_UNUSED(channel_id);
    WDT_Refresh(WDT);
    return 0;
}

static const struct wdt_driver_api wdt_mw320_api = {
    .setup = wdt_mw320_setup,
    .disable = wdt_mw320_disable,
    .install_timeout = wdt_mw320_install_timeout,
    .feed = wdt_mw320_feed,
};

static int wdt_mw320_init(const struct device* dev) { return 0; }

DEVICE_DT_INST_DEFINE(0, wdt_mw320_init, NULL, NULL, NULL, POST_KERNEL,
                      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &wdt_mw320_api);
