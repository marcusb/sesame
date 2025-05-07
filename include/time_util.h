#pragma once

#include <sys/time.h>

void setup_rtc(void);
int _gettimeofday(struct timeval *tv, void *tz);
long get_epoch_millis(void);
int hwrtc_time_set(time_t time);
unsigned tick_ms();