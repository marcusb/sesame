#include <zephyr/kernel.h>
#include <zephyr/drivers/watchdog.h>
#include "fsl_wdt.h"

#define DT_DRV_COMPAT nxp_mw320_wdt

static int wdt_mw320_setup(const struct device *dev, uint8_t options)
{
    wdt_config_t config;
    WDT_GetDefaultConfig(&config);
    /* 2^29 system clocks (~2.68s at 200MHz) */
    config.timeoutValue = kWDT_TimeoutVal2ToThePowerOf29;
    config.timeoutMode = kWDT_ModeTimeoutReset;
    config.enableWDT = true;
    
    WDT_Init(WDT, &config);
    return 0;
}

static int wdt_mw320_disable(const struct device *dev)
{
    /* Disable watchdog timer immediately */
    WDT->WDT_CR = 0;
    return 0;
}

static int wdt_mw320_install_timeout(const struct device *dev, const struct wdt_timeout_cfg *cfg)
{
    /* Just return 0 as channel ID, configuration is fixed for now */
    return 0;
}

static int wdt_mw320_feed(const struct device *dev, int channel_id)
{
    WDT_Refresh(WDT);
    return 0;
}

static const struct wdt_driver_api wdt_mw320_api = {
    .setup = wdt_mw320_setup,
    .disable = wdt_mw320_disable,
    .install_timeout = wdt_mw320_install_timeout,
    .feed = wdt_mw320_feed,
};

static int wdt_mw320_init(const struct device *dev)
{
    return 0;
}

DEVICE_DT_INST_DEFINE(0, wdt_mw320_init, NULL,
                      NULL, NULL,
                      POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
                      &wdt_mw320_api);
