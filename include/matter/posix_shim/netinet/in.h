/*
 * <netinet/in.h> POSIX shim over FreeRTOS+TCP for CHIP.
 *
 * Declarations-only. The implementation maps these to FreeRTOS+TCP
 * primitives in src/matter/posix_shim_impl.c (added in a later stage).
 *
 * Visible only to CHIP TUs via the include path order in chip.cmake.
 */
#pragma once

#include <stdint.h>
#include <sys/socket.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IPPROTO_IP   0
#define IPPROTO_ICMP 1
#define IPPROTO_TCP  6
#define IPPROTO_UDP  17
#define IPPROTO_IPV6 41
#define IPPROTO_ICMPV6 58

#define INET_ADDRSTRLEN  16
#define INET6_ADDRSTRLEN 46

#define INADDR_ANY       ((uint32_t) 0x00000000)
#define INADDR_BROADCAST ((uint32_t) 0xffffffff)
#define INADDR_LOOPBACK  ((uint32_t) 0x7f000001)

#define IPV6_ADD_MEMBERSHIP    12
#define IPV6_DROP_MEMBERSHIP   13
#define IPV6_JOIN_GROUP        IPV6_ADD_MEMBERSHIP
#define IPV6_LEAVE_GROUP       IPV6_DROP_MEMBERSHIP
#define IPV6_MULTICAST_IF      9
#define IPV6_MULTICAST_HOPS    10
#define IPV6_MULTICAST_LOOP    11
#define IPV6_UNICAST_HOPS      4
#define IPV6_V6ONLY            26
#define IPV6_RECVPKTINFO       49
#define IPV6_PKTINFO           50

#define IP_MULTICAST_IF        32
#define IP_MULTICAST_LOOP      34
#define IP_MULTICAST_TTL       33
#define IP_ADD_MEMBERSHIP      35
#define IP_DROP_MEMBERSHIP     36
#define IP_PKTINFO             8
#define IP_TOS                 1
#define IP_TTL                 2

struct in_addr {
    uint32_t s_addr;
};

struct in6_addr {
    union {
        uint8_t  u6_addr8[16];
        uint16_t u6_addr16[8];
        uint32_t u6_addr32[4];
    } in6_u;
#define s6_addr   in6_u.u6_addr8
#define s6_addr16 in6_u.u6_addr16
#define s6_addr32 in6_u.u6_addr32
};

struct sockaddr_in {
    sa_family_t    sin_family;
    uint16_t       sin_port;
    struct in_addr sin_addr;
    uint8_t        sin_zero[8];
};

struct sockaddr_in6 {
    sa_family_t     sin6_family;
    uint16_t        sin6_port;
    uint32_t        sin6_flowinfo;
    struct in6_addr sin6_addr;
    uint32_t        sin6_scope_id;
};

struct ipv6_mreq {
    struct in6_addr ipv6mr_multiaddr;
    unsigned int    ipv6mr_interface;
};

struct ip_mreq {
    struct in_addr imr_multiaddr;
    struct in_addr imr_interface;
};

struct in_pktinfo {
    int             ipi_ifindex;
    struct in_addr  ipi_spec_dst;
    struct in_addr  ipi_addr;
};

struct in6_pktinfo {
    struct in6_addr ipi6_addr;
    unsigned int    ipi6_ifindex;
};

extern const struct in6_addr in6addr_any;
extern const struct in6_addr in6addr_loopback;

uint16_t htons(uint16_t);
uint16_t ntohs(uint16_t);
uint32_t htonl(uint32_t);
uint32_t ntohl(uint32_t);

#ifdef __cplusplus
}
#endif
