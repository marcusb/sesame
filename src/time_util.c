#include "time_util.h"

#include <stdbool.h>
#include <stdint.h>
#include <sys/time.h>

// FreeRTOS
#include "FreeRTOS.h"
#include "timers.h"

// mw320
#ifndef QEMU
#include "fsl_rtc.h"
#endif

#define RTC_SIG 0xdeadbeef
/*
 * It is observed that it takes ~5ms for update, reset, start
 * cycle of the RTC. So we need to add the lost time everytime.
 * NOTE: This is an approximate compensation, so some inaccuracy
 * will persist in the time kept by the RTC.
 */
#define RTC_UPD_CALIB 5

__attribute((section(".nvram_uninit"))) uint64_t rtc_ticks;
__attribute((section(".nvram_uninit"))) static uint32_t rtc_sig;

static void update_clock(TimerHandle_t timer) {
#ifdef QEMU
    rtc_ticks += 10000 << 10;  // 10 seconds worth of ticks
#else
    rtc_ticks += (uint64_t)RTC_GetCounter(RTC) + RTC_UPD_CALIB;
    RTC_ResetTimer(RTC);
#endif
}

void setup_rtc() {
#ifndef QEMU
    // if RTC_SIG is found, it signifies a valid stored RTC count
    if (rtc_sig != RTC_SIG) {
        rtc_ticks = 0;
        rtc_sig = RTC_SIG;
    }

    rtc_config_t rtc_config;
    RTC_GetDefaultConfig(&rtc_config);
    RTC_Init(RTC, &rtc_config);
    RTC_EnableInterrupts(RTC, (uint32_t)kRTC_AlarmInterruptEnable);
    EnableIRQ(RTC_IRQn);
    RTC_ResetTimer(RTC);
    RTC_StartTimer(RTC);
#endif
}

void start_rtc_save(void) {
    // update the stored tick count from the RTC every 10 seconds
    static TimerHandle_t tm;
    tm = xTimerCreate("rtc", pdMS_TO_TICKS(10000), 1, NULL, update_clock);
    xTimerStart(tm, 0);
}

int gettimeofday(struct timeval* tv, void* tz) {
    if (tv == NULL) {
        return -1;
    }

#ifdef QEMU
    uint64_t ticks = rtc_ticks + ((uint64_t)xTaskGetTickCount() << 10) / 1000;
#else
    uint64_t ticks = rtc_ticks + RTC_GetCounter(RTC);
#endif
    tv->tv_sec = ticks >> 10;
    // 1 tick = 1/1024 ms = 1000/1024 us = 15625/16 us
    tv->tv_usec = ((ticks & 0x3ff) * 15625) >> 4;
    return 0;
}

long get_epoch_millis() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return 1000 * tv.tv_sec + tv.tv_usec / 1000;
}

uint32_t ulApplicationTimeHook(void) { return get_epoch_millis() / 1000; }

int hwrtc_time_set(time_t time) {
#ifndef QEMU
    RTC_ResetTimer(RTC);
#endif
    /// RTC runs at 1024 Hz
    rtc_ticks = (uint64_t)time << 10;
    return 0;
}

unsigned tick_ms() { return pdTICKS_TO_MS(xTaskGetTickCount()); }
