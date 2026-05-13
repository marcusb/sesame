/*
 * <sys/select.h> POSIX shim over FreeRTOS+TCP for CHIP.
 * Declarations-only.
 */
#pragma once

#include <sys/_select.h>

#ifdef __cplusplus
extern "C" {
#endif

int select(int nfds, fd_set *readfds, fd_set *writefds, fd_set *exceptfds,
           struct timeval *timeout);

#ifdef __cplusplus
}
#endif
