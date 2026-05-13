/*
 * <arpa/inet.h> POSIX shim over FreeRTOS+TCP for CHIP.
 * Declarations-only.
 */
#pragma once

#include <netinet/in.h>
#include <sys/socket.h>

#ifdef __cplusplus
extern "C" {
#endif

int         inet_pton(int af, const char *src, void *dst);
const char *inet_ntop(int af, const void *src, char *dst, socklen_t size);

#ifdef __cplusplus
}
#endif
