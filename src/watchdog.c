#include "watchdog.h"

#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(watchdog, LOG_LEVEL_INF);

static const struct device* const wdt = DEVICE_DT_GET(DT_NODELABEL(wdt0));
static int wdt_channel_id = -1;

void init_watchdog() {
    if (!device_is_ready(wdt)) {
        LOG_ERR("Watchdog device not ready");
        return;
    }

    struct wdt_timeout_cfg wdt_config = {
        .window.min = 0U,
        .window.max = 10000U,
        .callback = NULL,
        .flags = WDT_FLAG_RESET_SOC,
    };

    wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
    if (wdt_channel_id < 0) {
        LOG_ERR("Watchdog install error");
        return;
    }

    wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);
}

void feed_watchdog(void) {
    if (wdt_channel_id >= 0) {
        wdt_feed(wdt, wdt_channel_id);
    }
}
