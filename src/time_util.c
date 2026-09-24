#define _POSIX_C_SOURCE 200809L
#include "time_util.h"

#include <stdbool.h>
#include <stdint.h>
#include <sys/time.h>
#include <time.h>
#include <zephyr/device.h>
#include <zephyr/drivers/bbram.h>
#include <zephyr/drivers/counter.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/clock.h>

LOG_MODULE_REGISTER(time_util, LOG_LEVEL_INF);

#define RTC_SIG 0xdeadbeefU
/* Offset 0x10 is immediately following the 16 bytes reserved by Boot ROM
 * (0x00-0x0F) */
#define BBRAM_TIME_OFFSET 0x10U

struct nvram_time_data {
    uint32_t sig;
    uint32_t ticks_lo;
    uint32_t ticks_hi;
    uint32_t crc;
};

static uint64_t rtc_ticks = 0;
static bool s_time_initialized = false;
static struct k_work_delayable s_rtc_save_work;

static void nvram_save_ticks(uint64_t ticks) {
#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(bbram), nxp_mw320_bbram, okay)
    const struct device* bbram_dev = DEVICE_DT_GET(DT_NODELABEL(bbram));
    if (device_is_ready(bbram_dev)) {
        struct nvram_time_data nv;
        nv.sig = RTC_SIG;
        nv.ticks_lo = (uint32_t)(ticks & 0xFFFFFFFFU);
        nv.ticks_hi = (uint32_t)(ticks >> 32);
        nv.crc = RTC_SIG ^ nv.ticks_lo ^ nv.ticks_hi;
        bbram_write(bbram_dev, BBRAM_TIME_OFFSET, sizeof(nv),
                    (const uint8_t*)&nv);
    }
#endif
}

static bool nvram_load_ticks(uint64_t* ticks_out) {
#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(bbram), nxp_mw320_bbram, okay)
    const struct device* bbram_dev = DEVICE_DT_GET(DT_NODELABEL(bbram));
    if (device_is_ready(bbram_dev)) {
        struct nvram_time_data nv;
        if (bbram_read(bbram_dev, BBRAM_TIME_OFFSET, sizeof(nv),
                       (uint8_t*)&nv) == 0) {
            if (nv.sig == RTC_SIG &&
                nv.crc == (RTC_SIG ^ nv.ticks_lo ^ nv.ticks_hi)) {
                *ticks_out = ((uint64_t)nv.ticks_hi << 32) | nv.ticks_lo;
                return true;
            }
        }
    }
#endif
    return false;
}

static uint32_t get_counter_ticks(void) {
#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(rtc), nxp_mw320_rtc, okay)
    const struct device* rtc_dev = DEVICE_DT_GET(DT_NODELABEL(rtc));
    if (device_is_ready(rtc_dev)) {
        uint32_t val = 0;
        if (counter_get_value(rtc_dev, &val) == 0) {
            return val;
        }
    }
#endif
    return 0;
}

static void rtc_save_work_handler(struct k_work* work) {
    if (s_time_initialized) {
        uint32_t hw_cnt = get_counter_ticks();
        uint64_t total_ticks = rtc_ticks + hw_cnt;

        nvram_save_ticks(total_ticks);

        /* Re-align system clock with RTC crystal to prevent SysTick drift */
        struct timespec ts;
        ts.tv_sec = (time_t)(total_ticks >> 10);
        ts.tv_nsec = (long)(((total_ticks & 0x3FFU) * 1000000000ULL) >> 10);
        sys_clock_settime(SYS_CLOCK_REALTIME, &ts);
    }
    k_work_schedule(&s_rtc_save_work, K_SECONDS(10));
}

void setup_rtc(void) {
#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(rtc), nxp_mw320_rtc, okay)
    const struct device* rtc_dev = DEVICE_DT_GET(DT_NODELABEL(rtc));
    if (device_is_ready(rtc_dev)) {
        counter_reset(rtc_dev);
    }
#endif

    uint64_t saved_ticks = 0;
    if (nvram_load_ticks(&saved_ticks)) {
        rtc_ticks = saved_ticks;
        s_time_initialized = true;

        struct timespec ts;
        ts.tv_sec = (time_t)(rtc_ticks >> 10);
        ts.tv_nsec = (long)(((rtc_ticks & 0x3FFU) * 1000000000ULL) >> 10);
        sys_clock_settime(SYS_CLOCK_REALTIME, &ts);

        struct tm tm;
        time_t t = ts.tv_sec;
        gmtime_r(&t, &tm);
        char time_str[32];
        strftime(time_str, sizeof(time_str), "%Y-%m-%dT%H:%M:%SZ", &tm);
        LOG_INF("Restored time from RTC NVRAM: %s", time_str);
    } else {
        rtc_ticks = 0;
        s_time_initialized = false;
        LOG_INF("RTC NVRAM uninitialized or invalid, waiting for SNTP sync");
    }

    start_rtc_save();
}

void start_rtc_save(void) {
    k_work_init_delayable(&s_rtc_save_work, rtc_save_work_handler);
    k_work_schedule(&s_rtc_save_work, K_SECONDS(10));
}

int hwrtc_timespec_set(const struct timespec* ts) {
    if (ts == NULL) {
        return -EINVAL;
    }

#if DT_NODE_HAS_COMPAT_STATUS(DT_NODELABEL(rtc), nxp_mw320_rtc, okay)
    const struct device* rtc_dev = DEVICE_DT_GET(DT_NODELABEL(rtc));
    if (device_is_ready(rtc_dev)) {
        counter_reset(rtc_dev);
    }
#endif

    uint64_t sub_sec_ticks = ((uint64_t)ts->tv_nsec * 1024ULL) / 1000000000ULL;
    rtc_ticks = ((uint64_t)ts->tv_sec << 10) | (sub_sec_ticks & 0x3FFU);
    s_time_initialized = true;

    nvram_save_ticks(rtc_ticks);

    sys_clock_settime(SYS_CLOCK_REALTIME, ts);

    struct tm tm;
    time_t t = ts->tv_sec;
    gmtime_r(&t, &tm);
    char time_str[32];
    strftime(time_str, sizeof(time_str), "%Y-%m-%dT%H:%M:%SZ", &tm);
    LOG_INF("RTC time synchronized: %s", time_str);

    return 0;
}
