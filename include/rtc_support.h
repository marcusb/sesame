#pragma once

struct timeval;

void open_rtc(void);
int _gettimeofday(struct timeval *tv, void *tz);
long get_epoch_millis(void);