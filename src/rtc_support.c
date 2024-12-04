#include <stdint.h>
#include <sys/time.h>

#include "mdev_rtc.h"

static mdev_t *rtc_dev;

extern uint64_t rtc_ticks;

void open_rtc() { rtc_dev = rtc_drv_open("MDEV_RTC"); }

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
