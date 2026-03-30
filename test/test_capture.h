#pragma once

#include <stdint.h>
#include "logging.h"

#define CAPTURE_BUF_LEN    256
#define CAPTURE_MAX_MSGS   8

extern char    capture_msgs[CAPTURE_MAX_MSGS][CAPTURE_BUF_LEN];
extern uint8_t capture_levels[CAPTURE_MAX_MSGS];
extern int     capture_count;

void capture_backend(const log_msg_t *log);
void capture_reset(void);
/* Yield 50 ms so the async logging task drains its queue, then reset. */
void capture_drain_and_reset(void);
