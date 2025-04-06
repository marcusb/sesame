#include "time_util.h"

#include <stdint.h>
#include <sys/time.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "timers.h"

// Application
#include "mdev_rtc.h"
#include "rtc.h"

extern uint64_t rtc_ticks;

static mdev_t *rtc_dev;

static void update_clock(TimerHandle_t timer) {
    hwrtc_time_update();
    rtc_drv_reset(rtc_dev);
}

void setup_rtc(void) {
    rtc_dev = rtc_drv_open("MDEV_RTC");
    // update the stored tick count from the RTC every 10 seconds
    TimerHandle_t tm =
        xTimerCreate("rtc", pdMS_TO_TICKS(10000), true, NULL, update_clock);
    xTimerStart(tm, 0);
}

int _gettimeofday(struct timeval *tv, void *tz) {
    if (tv == NULL) {
        return -1;
    }

    uint64_t ticks = rtc_ticks + (uint64_t)rtc_drv_get(rtc_dev);
    tv->tv_sec = ticks >> 10;
    /* fixme; This could be improved */
    tv->tv_usec = (ticks & 0x3ff) * 1000000 / 1024;

    /* Unix value for successful */
    return 0;
}

long get_epoch_millis() {
    struct timeval tv;
    _gettimeofday(&tv, NULL);
    return 1000 * tv.tv_sec + tv.tv_usec / 1000;
}

unsigned tick_ms() { return pdTICKS_TO_MS(xTaskGetTickCount()); }
