#pragma once

#include <stdint.h>

#define INET_CONFIG_ERROR_TYPE int32_t
#define INET_CONFIG_NO_ERROR   0
#define INET_CONFIG_ERROR_MIN  1000000
#define INET_CONFIG_ERROR_MAX  1000999

/* IPv4 disabled on QEMU: FreeRTOS+TCP lacks IPV6_V6ONLY so the IPv6 socket
 * captures both families, causing EADDRINUSE when CHIP tries to bind a
 * separate IPv4 socket to the same port. */
#ifdef USE_QEMU
#define INET_CONFIG_ENABLE_IPV4 0
#else
#define INET_CONFIG_ENABLE_IPV4 1
#endif

#ifndef INET_CONFIG_NUM_TCP_ENDPOINTS
#define INET_CONFIG_NUM_TCP_ENDPOINTS 4
#endif

#ifndef INET_CONFIG_NUM_UDP_ENDPOINTS
#define INET_CONFIG_NUM_UDP_ENDPOINTS 4
#endif
