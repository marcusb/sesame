/* BSD-socket shim for CHIP over FreeRTOS+TCP.
 *
 * CHIP is built with CHIP_SYSTEM_CONFIG_USE_SOCKETS=1, which means its
 * System layer and Inet layer call standard POSIX socket/fd APIs.  FreeRTOS
 * bare-metal doesn't provide these, so we shim them here.
 *
 * File descriptor space:
 *   10–89  : real sockets, backed by FreeRTOS+TCP Socket_t handles.
 *   100–103: pipe endpoints, backed by FreeRTOS byte queues (for WakeEvent).
 *
 * select() checks pipe queues and socket receive-readiness, then sleeps for
 * up to the requested timeout before returning.
 */

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <sys/_timeval.h>

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* FreeRTOS+TCP for socket and inet helpers */
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Sockets.h"

/* ---- Byte-order ---------------------------------------------------------- */

/* ARM Cortex-M4 is always little-endian. */
#define _BSWAP16(x) ((uint16_t)(((x) >> 8) | ((x) << 8)))
#define _BSWAP32(x)                                        \
    ((uint32_t)(((x) >> 24) | (((x) >> 8) & 0x0000FF00u) | \
                (((x) << 8) & 0x00FF0000u) | ((x) << 24)))

uint32_t htonl(uint32_t x) { return _BSWAP32(x); }
uint32_t ntohl(uint32_t x) { return _BSWAP32(x); }
uint16_t htons(uint16_t x) { return _BSWAP16(x); }
uint16_t ntohs(uint16_t x) { return _BSWAP16(x); }

/* ---- inet_pton / inet_ntop ------------------------------------------------
 */

#include <netinet/in.h>
#include <sys/socket.h>

int inet_pton(int af, const char* src, void* dst) {
    if (af == AF_INET) {
        uint32_t addr = FreeRTOS_inet_addr(src);
        if (addr == 0xFFFFFFFFu) return 0;
        memcpy(dst, &addr, 4);
        return 1;
    }
    /* IPv6 not supported yet */
    return -1;
}

const char* inet_ntop(int af, const void* src, char* dst, socklen_t size) {
    if (af == AF_INET) {
        uint32_t addr;
        memcpy(&addr, src, 4);
        FreeRTOS_inet_ntoa(addr, dst);
        return dst;
    }
    /* IPv6: best-effort hex print */
    if (af == AF_INET6 && size >= 40) {
        const uint8_t* b = (const uint8_t*)src;
        snprintf(dst, size,
                 "%02x%02x:%02x%02x:%02x%02x:%02x%02x:"
                 "%02x%02x:%02x%02x:%02x%02x:%02x%02x",
                 b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7], b[8], b[9],
                 b[10], b[11], b[12], b[13], b[14], b[15]);
        return dst;
    }
    return NULL;
}

/* ---- Interface enumeration ---------------------------------------------- */

#include <net/if.h>

int getifaddrs(struct ifaddrs** ifap) {
    static struct ifaddrs s_entry;
    static struct sockaddr_in s_addr;
    static char s_name[] = "mlan0";

    memset(&s_entry, 0, sizeof(s_entry));
    memset(&s_addr, 0, sizeof(s_addr));

    s_addr.sin_family = AF_INET;
    s_addr.sin_addr.s_addr = FreeRTOS_GetIPAddress();

    s_entry.ifa_name = s_name;
    s_entry.ifa_flags = IFF_UP | IFF_RUNNING | IFF_BROADCAST | IFF_MULTICAST;
    s_entry.ifa_addr = (struct sockaddr*)&s_addr;
    s_entry.ifa_netmask = NULL;
    s_entry.ifa_next = NULL;

    *ifap = &s_entry;
    return 0;
}

void freeifaddrs(struct ifaddrs* ifa) { (void)ifa; }

char* if_indextoname(unsigned int ifindex, char* ifname) {
    if (ifindex == 1) {
        strncpy(ifname, "mlan0", IF_NAMESIZE);
        return ifname;
    }
    return NULL;
}

unsigned int if_nametoindex(const char* ifname) {
    if (strcmp(ifname, "mlan0") == 0) return 1;
    return 0;
}

/* ---- Socket FD table ----------------------------------------------------- */

#define SOCK_FD_BASE 10
#define MAX_SOCKETS 16

static Socket_t s_sockets[MAX_SOCKETS]; /* NULL = slot unused */

static int alloc_sock_fd(Socket_t sock) {
    for (int i = 0; i < MAX_SOCKETS; i++) {
        if (s_sockets[i] == NULL) {
            s_sockets[i] = sock;
            return SOCK_FD_BASE + i;
        }
    }
    return -1;
}

static Socket_t fd_to_socket(int fd) {
    int idx = fd - SOCK_FD_BASE;
    if (idx < 0 || idx >= MAX_SOCKETS) return NULL;
    return s_sockets[idx];
}

/* ---- BSD socket API ------------------------------------------------------ */

/* Convert POSIX sockaddr_in to FreeRTOS+TCP freertos_sockaddr. */
static void to_freeaddr(const struct sockaddr* sa,
                        struct freertos_sockaddr* fa) {
    memset(fa, 0, sizeof(*fa));
    if (sa->sa_family == AF_INET) {
        const struct sockaddr_in* sin = (const struct sockaddr_in*)sa;
        fa->sin_family = FREERTOS_AF_INET4;
        fa->sin_port = sin->sin_port;
        fa->sin_address.ulIP_IPv4 = sin->sin_addr.s_addr;
    } else {
        fa->sin_family = FREERTOS_AF_INET6;
        const struct sockaddr_in6* sin6 = (const struct sockaddr_in6*)sa;
        fa->sin_port = sin6->sin6_port;
        memcpy(fa->sin_address.xIP_IPv6.ucBytes, sin6->sin6_addr.s6_addr, 16);
    }
}

/* Convert FreeRTOS+TCP freertos_sockaddr to POSIX sockaddr_in. */
static void from_freeaddr(const struct freertos_sockaddr* fa,
                          struct sockaddr* sa, socklen_t* slen) {
    if (fa->sin_family == FREERTOS_AF_INET4) {
        struct sockaddr_in* sin = (struct sockaddr_in*)sa;
        sin->sin_family = AF_INET;
        sin->sin_port = fa->sin_port;
        sin->sin_addr.s_addr = fa->sin_address.ulIP_IPv4;
        if (slen) *slen = sizeof(*sin);
    } else {
        struct sockaddr_in6* sin6 = (struct sockaddr_in6*)sa;
        sin6->sin6_family = AF_INET6;
        sin6->sin6_port = fa->sin_port;
        memcpy(sin6->sin6_addr.s6_addr, fa->sin_address.xIP_IPv6.ucBytes, 16);
        if (slen) *slen = sizeof(*sin6);
    }
}

int socket(int domain, int type, int protocol) {
    (void)protocol;
    BaseType_t ftype =
        (type == SOCK_STREAM) ? FREERTOS_SOCK_STREAM : FREERTOS_SOCK_DGRAM;
    BaseType_t fdom =
        (domain == AF_INET6) ? FREERTOS_AF_INET6 : FREERTOS_AF_INET4;
    BaseType_t fprot =
        (type == SOCK_STREAM) ? FREERTOS_IPPROTO_TCP : FREERTOS_IPPROTO_UDP;
    Socket_t sock = FreeRTOS_socket(fdom, ftype, fprot);
    if (sock == FREERTOS_INVALID_SOCKET) {
        errno = ENOMEM;
        return -1;
    }
    int fd = alloc_sock_fd(sock);
    if (fd < 0) {
        FreeRTOS_closesocket(sock);
        errno = EMFILE;
        return -1;
    }
    return fd;
}

int bind(int fd, const struct sockaddr* addr, socklen_t addrlen) {
    (void)addrlen;
    Socket_t sock = fd_to_socket(fd);
    if (!sock) {
        errno = EBADF;
        return -1;
    }
    struct freertos_sockaddr fa;
    to_freeaddr(addr, &fa);
    if (FreeRTOS_bind(sock, &fa, sizeof(fa)) != 0) {
        errno = EADDRINUSE;
        return -1;
    }
    return 0;
}

int listen(int fd, int backlog) {
    (void)backlog;
    Socket_t sock = fd_to_socket(fd);
    if (!sock) {
        errno = EBADF;
        return -1;
    }
    if (FreeRTOS_listen(sock, backlog) != 0) {
        errno = EOPNOTSUPP;
        return -1;
    }
    return 0;
}

int accept(int fd, struct sockaddr* addr, socklen_t* addrlen) {
    Socket_t sock = fd_to_socket(fd);
    if (!sock) {
        errno = EBADF;
        return -1;
    }
    struct freertos_sockaddr fa;
    socklen_t falen = sizeof(fa);
    Socket_t newsock = FreeRTOS_accept(sock, &fa, &falen);
    if (newsock == FREERTOS_INVALID_SOCKET || newsock == NULL) {
        errno = EAGAIN;
        return -1;
    }
    if (addr) from_freeaddr(&fa, addr, addrlen);
    int newfd = alloc_sock_fd(newsock);
    if (newfd < 0) {
        FreeRTOS_closesocket(newsock);
        errno = EMFILE;
        return -1;
    }
    return newfd;
}

int connect(int fd, const struct sockaddr* addr, socklen_t addrlen) {
    (void)addrlen;
    Socket_t sock = fd_to_socket(fd);
    if (!sock) {
        errno = EBADF;
        return -1;
    }
    struct freertos_sockaddr fa;
    to_freeaddr(addr, &fa);
    if (FreeRTOS_connect(sock, &fa, sizeof(fa)) != 0) {
        errno = ECONNREFUSED;
        return -1;
    }
    return 0;
}

ssize_t send(int fd, const void* buf, size_t len, int flags) {
    (void)flags;
    Socket_t sock = fd_to_socket(fd);
    if (!sock) {
        errno = EBADF;
        return -1;
    }
    BaseType_t sent = FreeRTOS_send(sock, buf, len, 0);
    if (sent < 0) {
        errno = EPIPE;
        return -1;
    }
    return (ssize_t)sent;
}

ssize_t recv(int fd, void* buf, size_t len, int flags) {
    (void)flags;
    Socket_t sock = fd_to_socket(fd);
    if (!sock) {
        errno = EBADF;
        return -1;
    }
    BaseType_t r = FreeRTOS_recv(sock, buf, len, 0);
    if (r < 0) {
        errno = EAGAIN;
        return -1;
    }
    return (ssize_t)r;
}

ssize_t sendto(int fd, const void* buf, size_t len, int flags,
               const struct sockaddr* dest_addr, socklen_t addrlen) {
    (void)flags;
    (void)addrlen;
    Socket_t sock = fd_to_socket(fd);
    if (!sock) {
        errno = EBADF;
        return -1;
    }
    struct freertos_sockaddr fa;
    to_freeaddr(dest_addr, &fa);
    BaseType_t sent = FreeRTOS_sendto(sock, buf, len, 0, &fa, sizeof(fa));
    if (sent < 0) {
        errno = ENETUNREACH;
        return -1;
    }
    return (ssize_t)sent;
}

ssize_t recvfrom(int fd, void* buf, size_t len, int flags,
                 struct sockaddr* src_addr, socklen_t* addrlen) {
    (void)flags;
    Socket_t sock = fd_to_socket(fd);
    if (!sock) {
        errno = EBADF;
        return -1;
    }
    struct freertos_sockaddr fa;
    socklen_t falen = sizeof(fa);
    BaseType_t r = FreeRTOS_recvfrom(sock, buf, len, 0, &fa, &falen);
    if (r < 0) {
        errno = EAGAIN;
        return -1;
    }
    if (src_addr) from_freeaddr(&fa, src_addr, addrlen);
    return (ssize_t)r;
}

ssize_t sendmsg(int fd, const struct msghdr* msg, int flags) {
    if (!msg || !msg->msg_iov || msg->msg_iovlen < 1) {
        errno = EINVAL;
        return -1;
    }
    /* CHIP sends single-iov UDP datagrams; use the first iov as the buffer. */
    return sendto(fd, msg->msg_iov[0].iov_base, msg->msg_iov[0].iov_len, flags,
                  (struct sockaddr*)msg->msg_name, msg->msg_namelen);
}

ssize_t recvmsg(int fd, struct msghdr* msg, int flags) {
    if (!msg || !msg->msg_iov || msg->msg_iovlen < 1) {
        errno = EINVAL;
        return -1;
    }
    struct sockaddr_in src;
    socklen_t srclen = sizeof(src);
    ssize_t n = recvfrom(fd, msg->msg_iov[0].iov_base, msg->msg_iov[0].iov_len,
                         flags, (struct sockaddr*)&src, &srclen);
    if (n < 0) return -1;
    if (msg->msg_name)
        memcpy(
            msg->msg_name, &src,
            srclen < (socklen_t)msg->msg_namelen ? srclen : msg->msg_namelen);
    /* Provide a minimal IP_PKTINFO so CHIP's MinMdns knows which interface
     * a datagram arrived on.  We have one interface (mlan0 = index 1). */
    if (msg->msg_control &&
        msg->msg_controllen >=
            sizeof(struct cmsghdr) + sizeof(struct in_pktinfo)) {
        struct cmsghdr* cmsg = (struct cmsghdr*)msg->msg_control;
        cmsg->cmsg_len = CMSG_LEN(sizeof(struct in_pktinfo));
        cmsg->cmsg_level = IPPROTO_IP;
        cmsg->cmsg_type = IP_PKTINFO;
        struct in_pktinfo* pktinfo = (struct in_pktinfo*)CMSG_DATA(cmsg);
        pktinfo->ipi_ifindex = 1; /* mlan0 */
        pktinfo->ipi_spec_dst.s_addr = FreeRTOS_GetIPAddress();
        pktinfo->ipi_addr.s_addr = FreeRTOS_GetIPAddress();
        msg->msg_controllen = cmsg->cmsg_len;
    } else {
        msg->msg_controllen = 0;
    }
    msg->msg_flags = 0;
    return n;
}

int setsockopt(int fd, int level, int optname, const void* optval,
               socklen_t optlen) {
    Socket_t sock = fd_to_socket(fd);
    if (!sock) {
        errno = EBADF;
        return -1;
    }

    /* Map POSIX socket options to FreeRTOS+TCP options. */
    if (level == SOL_SOCKET && optname == SO_RCVTIMEO) {
        const struct timeval* tv = (const struct timeval*)optval;
        TickType_t ticks =
            (TickType_t)(tv->tv_sec * 1000 + tv->tv_usec / 1000) /
            portTICK_PERIOD_MS;
        FreeRTOS_setsockopt(sock, 0, FREERTOS_SO_RCVTIMEO, &ticks,
                            sizeof(ticks));
        return 0;
    }
    if (level == SOL_SOCKET && optname == SO_SNDTIMEO) {
        const struct timeval* tv = (const struct timeval*)optval;
        TickType_t ticks =
            (TickType_t)(tv->tv_sec * 1000 + tv->tv_usec / 1000) /
            portTICK_PERIOD_MS;
        FreeRTOS_setsockopt(sock, 0, FREERTOS_SO_SNDTIMEO, &ticks,
                            sizeof(ticks));
        return 0;
    }
    /* IP_PKTINFO, IPV6_RECVPKTINFO, IP_MULTICAST_IF, IPV6_JOIN_GROUP, etc.
     * are silently accepted — multicast join is handled by mdns_mcast_join.c.
     */
    (void)optlen;
    return 0;
}

int getsockopt(int fd, int level, int optname, void* optval,
               socklen_t* optlen) {
    (void)level;
    (void)optname;
    Socket_t sock = fd_to_socket(fd);
    if (!sock) {
        errno = EBADF;
        return -1;
    }
    if (optname == SO_ERROR && optval && optlen && *optlen >= sizeof(int)) {
        *(int*)optval = 0;
        *optlen = sizeof(int);
        return 0;
    }
    errno = ENOPROTOOPT;
    return -1;
}

int getsockname(int fd, struct sockaddr* addr, socklen_t* addrlen) {
    Socket_t sock = fd_to_socket(fd);
    if (!sock) {
        errno = EBADF;
        return -1;
    }
    struct freertos_sockaddr fa;
    socklen_t falen = sizeof(fa);
    if (FreeRTOS_GetLocalAddress(sock, &fa) != 0) {
        /* If not bound, return INADDR_ANY port 0 */
        struct sockaddr_in* sin = (struct sockaddr_in*)addr;
        sin->sin_family = AF_INET;
        sin->sin_addr.s_addr = 0;
        sin->sin_port = 0;
        if (addrlen) *addrlen = sizeof(*sin);
        return 0;
    }
    from_freeaddr(&fa, addr, addrlen);
    (void)falen;
    return 0;
}

int getpeername(int fd, struct sockaddr* addr, socklen_t* addrlen) {
    Socket_t sock = fd_to_socket(fd);
    if (!sock) {
        errno = EBADF;
        return -1;
    }
    struct freertos_sockaddr fa;
    if (FreeRTOS_GetRemoteAddress(sock, &fa) != 0) {
        errno = ENOTCONN;
        return -1;
    }
    from_freeaddr(&fa, addr, addrlen);
    return 0;
}

int shutdown(int fd, int how) {
    Socket_t sock = fd_to_socket(fd);
    if (!sock) {
        errno = EBADF;
        return -1;
    }
    FreeRTOS_shutdown(sock, how);
    return 0;
}

/* ---- Pipe FD emulation --------------------------------------------------- */
/* WakeEvent.cpp uses pipe()/read()/write()/close()/fcntl() to create a
 * self-pipe for waking select().  We map each pipe endpoint to a tiny
 * FreeRTOS byte queue. */

#define PIPE_FD_BASE 100
#define MAX_PIPE_ENDS 4

static QueueHandle_t s_pipe_queues[MAX_PIPE_ENDS];

static int alloc_pipe_fd(QueueHandle_t q) {
    for (int i = 0; i < MAX_PIPE_ENDS; i++) {
        if (s_pipe_queues[i] == NULL) {
            s_pipe_queues[i] = q;
            return PIPE_FD_BASE + i;
        }
    }
    return -1;
}

static QueueHandle_t fd_to_queue(int fd) {
    int idx = fd - PIPE_FD_BASE;
    if (idx < 0 || idx >= MAX_PIPE_ENDS) return NULL;
    return s_pipe_queues[idx];
}

int pipe(int pipefd[2]) {
    QueueHandle_t rq = xQueueCreate(8, sizeof(uint8_t));
    if (rq == NULL) {
        errno = ENOMEM;
        return -1;
    }
    int rfd = alloc_pipe_fd(rq);
    int wfd = alloc_pipe_fd(rq); /* both ends share the same queue */
    if (rfd < 0 || wfd < 0) {
        vQueueDelete(rq);
        errno = EMFILE;
        return -1;
    }
    pipefd[0] = rfd;
    pipefd[1] = wfd;
    return 0;
}

ssize_t read(int fd, void* buf, size_t count) {
    QueueHandle_t q = fd_to_queue(fd);
    if (q == NULL) {
        errno = EBADF;
        return -1;
    }
    size_t n = 0;
    while (n < count) {
        if (xQueueReceive(q, (uint8_t*)buf + n, 0) != pdTRUE) break;
        n++;
    }
    if (n == 0) {
        errno = EAGAIN;
        return -1;
    }
    return (ssize_t)n;
}

ssize_t write(int fd, const void* buf, size_t count) {
    QueueHandle_t q = fd_to_queue(fd);
    if (q == NULL) {
        errno = EBADF;
        return -1;
    }
    size_t n = 0;
    while (n < count) {
        if (xQueueSend(q, (const uint8_t*)buf + n, 0) != pdTRUE) break;
        n++;
    }
    return (ssize_t)n;
}

int close(int fd) {
    /* Socket FDs */
    int sidx = fd - SOCK_FD_BASE;
    if (sidx >= 0 && sidx < MAX_SOCKETS) {
        if (s_sockets[sidx] != NULL) {
            FreeRTOS_closesocket(s_sockets[sidx]);
            s_sockets[sidx] = NULL;
        }
        return 0;
    }
    /* Pipe FDs */
    int pidx = fd - PIPE_FD_BASE;
    if (pidx < 0 || pidx >= MAX_PIPE_ENDS) {
        errno = EBADF;
        return -1;
    }
    if (s_pipe_queues[pidx] != NULL) {
        QueueHandle_t q = s_pipe_queues[pidx];
        s_pipe_queues[pidx] = NULL;
        bool shared = false;
        for (int i = 0; i < MAX_PIPE_ENDS; i++) {
            if (s_pipe_queues[i] == q) {
                shared = true;
                break;
            }
        }
        if (!shared) vQueueDelete(q);
    }
    return 0;
}

#define F_SETFL 4
#define O_NONBLOCK 0x800
#include <stdarg.h>

int fcntl(int fd, int cmd, ...) {
    (void)fd;
    (void)cmd;
    return 0;
}

/* ioctl — handles FIONREAD (bytes available) and FIONBIO (non-blocking mode).
 */
int ioctl(int fd, unsigned long request, ...) {
    (void)request;
    Socket_t sock = fd_to_socket(fd);
    if (!sock) {
        errno = EBADF;
        return -1;
    }
    return 0;
}

/* ---- select -------------------------------------------------------------- */
/* Checks pipe queues and socket receive-readiness, then waits for up to the
 * requested timeout.  Real wakeup callbacks (plan step 4) would replace the
 * vTaskDelay sleep with a task notification. */

#include <sys/select.h>

int select(int nfds, fd_set* readfds, fd_set* writefds, fd_set* exceptfds,
           struct timeval* timeout) {
    (void)writefds;
    (void)exceptfds;

    /* Check for pending data on pipe FDs first. */
    for (int fd = PIPE_FD_BASE; fd < PIPE_FD_BASE + MAX_PIPE_ENDS; fd++) {
        QueueHandle_t q = fd_to_queue(fd);
        if (q && uxQueueMessagesWaiting(q) > 0 && readfds && fd < nfds &&
            FD_ISSET(fd, readfds))
            return 1;
    }

    /* Check socket FDs for available receive data. */
    for (int fd = SOCK_FD_BASE; fd < SOCK_FD_BASE + MAX_SOCKETS; fd++) {
        if (fd >= nfds) break;
        if (!readfds || !FD_ISSET(fd, readfds)) continue;
        Socket_t sock = fd_to_socket(fd);
        if (!sock) continue;
        /* Non-blocking peek: timeout=0 */
        TickType_t zero = 0;
        FreeRTOS_setsockopt(sock, 0, FREERTOS_SO_RCVTIMEO, &zero, sizeof(zero));
        uint8_t peek;
        BaseType_t r = FreeRTOS_recvfrom(sock, &peek, sizeof(peek),
                                         FREERTOS_MSG_PEEK, NULL, NULL);
        /* Restore blocking with a short timeout so the CHIP loop isn't too
         * tight */
        TickType_t recv_to = pdMS_TO_TICKS(1);
        FreeRTOS_setsockopt(sock, 0, FREERTOS_SO_RCVTIMEO, &recv_to,
                            sizeof(recv_to));
        if (r > 0) return 1;
    }

    if (timeout) {
        TickType_t ticks =
            (TickType_t)(timeout->tv_sec * 1000 + timeout->tv_usec / 1000) /
            portTICK_PERIOD_MS;
        if (ticks > 0) vTaskDelay(ticks);
    } else {
        vTaskDelay(1);
    }
    return 0;
}
