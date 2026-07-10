#include <zephyr/device.h>
#include <zephyr/drivers/watchdog.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

void main(void) {
    const struct device* const wdt = DEVICE_DT_GET(DT_NODELABEL(wdt0));

    if (!device_is_ready(wdt)) {
        printk("Watchdog device not ready\n");
        return;
    }

    struct wdt_timeout_cfg wdt_config = {
        .window.min = 0U,
        .window.max = 2000U,
        .callback = NULL,
        .flags = WDT_FLAG_RESET_SOC,
    };

    int wdt_channel_id = wdt_install_timeout(wdt, &wdt_config);
    if (wdt_channel_id < 0) {
        printk("Watchdog install error\n");
        return;
    }

    wdt_setup(wdt, WDT_OPT_PAUSE_HALTED_BY_DBG);

    while (1) {
        wdt_feed(wdt, wdt_channel_id);
        k_sleep(K_MSEC(1000));
    }
}
