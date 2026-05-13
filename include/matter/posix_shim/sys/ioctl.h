/*
 * <sys/ioctl.h> POSIX shim over FreeRTOS+TCP for CHIP.
 * Declarations-only.
 */
#pragma once

#include <sys/socket.h>

#ifdef __cplusplus
extern "C" {
#endif

/* TTY / socket ioctl constants */
#define TIOCOUTQ      0x5411

/* Socket interface flags (used by SIOCGIFFLAGS) */
#define SIOCGIFFLAGS  0x8913
#define SIOCSIFFLAGS  0x8914
#define SIOCGIFADDR   0x8915
#define SIOCGIFNETMASK 0x891b
#define SIOCGIFHWADDR  0x8927
#define SIOCGIFINDEX   0x8933

struct ifreq {
    char ifr_name[16];
    union {
        struct sockaddr ifr_addr;
        struct sockaddr ifr_netmask;
        struct sockaddr ifr_hwaddr;
        short           ifr_flags;
        int             ifr_ifindex;
    };
};

int ioctl(int fd, unsigned long request, ...);

#ifdef __cplusplus
}
#endif
