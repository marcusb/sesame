#pragma once

struct timeval;

void setup_rtc(void);
int _gettimeofday(struct timeval *tv, void *tz);
long get_epoch_millis(void);
unsigned tick_ms();
