#include "time_util.h"

#include <time.h>

#include "FreeRTOS.h"
#include "task.h"

uint64_t rtc_ticks;

void setup_rtc(void) {}
void start_rtc_save(void) {}
int hwrtc_time_set(time_t t) {
    (void)t;
    return 0;
}

int _gettimeofday(struct timeval* tv, void* tz) {
    (void)tz;
    if (tv) {
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        tv->tv_sec = ts.tv_sec;
        tv->tv_usec = ts.tv_nsec / 1000;
    }
    return 0;
}

long get_epoch_millis(void) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    return (long)(ts.tv_sec * 1000 + ts.tv_nsec / 1000000);
}

unsigned tick_ms(void) {
    return (unsigned)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}
