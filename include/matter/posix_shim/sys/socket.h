/*
 * <sys/socket.h> POSIX shim over FreeRTOS+TCP for CHIP.
 * Declarations-only.
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include <sys/types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint16_t sa_family_t;
typedef uint32_t socklen_t;

#define AF_UNSPEC 0
#define AF_INET   2
#define AF_INET6  10

#define PF_UNSPEC AF_UNSPEC
#define PF_INET   AF_INET
#define PF_INET6  AF_INET6

#define SOCK_STREAM 1
#define SOCK_DGRAM  2
#define SOCK_RAW    3

#define SOL_SOCKET 0xFFFF

#define SO_REUSEADDR 0x0004
#define SO_BROADCAST 0x0020
#define SO_RCVTIMEO  0x1006
#define SO_SNDTIMEO  0x1005
#define SO_ERROR     0x1007
#define SO_KEEPALIVE 0x0008

#define MSG_PEEK      0x01
#define MSG_DONTWAIT  0x40
#define MSG_TRUNC     0x20
#define MSG_CTRUNC    0x08

#define SHUT_RD   0
#define SHUT_WR   1
#define SHUT_RDWR 2

struct sockaddr {
    sa_family_t sa_family;
    char        sa_data[14];
};

struct sockaddr_storage {
    sa_family_t ss_family;
    char        __ss_padding[126];
};

struct iovec {
    void  *iov_base;
    size_t iov_len;
};

struct msghdr {
    void         *msg_name;
    socklen_t     msg_namelen;
    struct iovec *msg_iov;
    int           msg_iovlen;
    void         *msg_control;
    socklen_t     msg_controllen;
    int           msg_flags;
};

struct cmsghdr {
    socklen_t cmsg_len;
    int       cmsg_level;
    int       cmsg_type;
};

#define CMSG_ALIGN(len) (((len) + sizeof(long) - 1) & ~(sizeof(long) - 1))
#define CMSG_SPACE(len) (CMSG_ALIGN(len) + CMSG_ALIGN(sizeof(struct cmsghdr)))
#define CMSG_LEN(len)   (CMSG_ALIGN(sizeof(struct cmsghdr)) + (len))
#define CMSG_DATA(cmsg) ((unsigned char *)((cmsg) + 1))
#define CMSG_FIRSTHDR(mhdr) \
    ((mhdr)->msg_controllen >= sizeof(struct cmsghdr) ? \
     (struct cmsghdr *)(mhdr)->msg_control : NULL)
#define CMSG_NXTHDR(mhdr, cmsg) ((struct cmsghdr *) NULL)

int       socket(int domain, int type, int protocol);
int       bind(int sockfd, const struct sockaddr *addr, socklen_t addrlen);
int       listen(int sockfd, int backlog);
int       accept(int sockfd, struct sockaddr *addr, socklen_t *addrlen);
int       connect(int sockfd, const struct sockaddr *addr, socklen_t addrlen);
ssize_t   send(int sockfd, const void *buf, size_t len, int flags);
ssize_t   recv(int sockfd, void *buf, size_t len, int flags);
ssize_t   sendto(int sockfd, const void *buf, size_t len, int flags,
                 const struct sockaddr *dest_addr, socklen_t addrlen);
ssize_t   recvfrom(int sockfd, void *buf, size_t len, int flags,
                   struct sockaddr *src_addr, socklen_t *addrlen);
ssize_t   sendmsg(int sockfd, const struct msghdr *msg, int flags);
ssize_t   recvmsg(int sockfd, struct msghdr *msg, int flags);
int       setsockopt(int sockfd, int level, int optname,
                     const void *optval, socklen_t optlen);
int       getsockopt(int sockfd, int level, int optname,
                     void *optval, socklen_t *optlen);
int       getsockname(int sockfd, struct sockaddr *addr, socklen_t *addrlen);
int       shutdown(int sockfd, int how);

#ifdef __cplusplus
}
#endif
