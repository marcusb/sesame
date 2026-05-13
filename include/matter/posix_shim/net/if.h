/*
 * <net/if.h> POSIX shim over FreeRTOS+TCP for CHIP.
 * Declarations-only.
 */
#pragma once

#include <sys/socket.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IF_NAMESIZE 16
#define IFNAMSIZ    IF_NAMESIZE

struct ifaddrs {
    struct ifaddrs  *ifa_next;
    char            *ifa_name;
    unsigned int     ifa_flags;
    struct sockaddr *ifa_addr;
    struct sockaddr *ifa_netmask;
    struct sockaddr *ifa_broadaddr;
    struct sockaddr *ifa_dstaddr;
    void            *ifa_data;
};

#define IFF_UP        0x1
#define IFF_LOOPBACK  0x8
#define IFF_RUNNING   0x40
#define IFF_BROADCAST 0x2
#define IFF_MULTICAST 0x1000

struct if_nameindex {
    unsigned int  if_index;
    char         *if_name;
};

unsigned int        if_nametoindex(const char *ifname);
char               *if_indextoname(unsigned int ifindex, char *ifname);
struct if_nameindex *if_nameindex(void);
void                if_freenameindex(struct if_nameindex *ptr);
int                 getifaddrs(struct ifaddrs **ifap);
void                freeifaddrs(struct ifaddrs *ifa);

#ifdef __cplusplus
}
#endif
