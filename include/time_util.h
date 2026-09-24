#pragma once

#include <stdint.h>
#include <sys/time.h>
#include <time.h>

void setup_rtc(void);
void start_rtc_save(void);
int hwrtc_timespec_set(const struct timespec* ts);
