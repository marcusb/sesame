/*
 * LD_PRELOAD shim that intercepts libslirp's slirp_new() and adds a
 * host→guest port forward immediately after creation.
 *
 * Build as a shared library and inject via LD_PRELOAD.
 *
 * Environment:
 *   HARNESS_HOST_PORT   TCP/UDP port on 127.0.0.1 the test driver connects to
 *   HARNESS_GUEST_PORT  Matching port inside the FreeRTOS stack
 *   HARNESS_GUEST_UDP   "1" → UDP forward, else TCP
 */

#define _GNU_SOURCE
#include <arpa/inet.h>
#include <dlfcn.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>

#include <libslirp.h>

Slirp *slirp_new(const SlirpConfig *cfg,
                 const SlirpCb *callbacks,
                 void *opaque) {
    Slirp *(*real_slirp_new)(const SlirpConfig *, const SlirpCb *, void *);
    real_slirp_new = dlsym(RTLD_NEXT, "slirp_new");
    if (!real_slirp_new) {
        fprintf(stderr, "slirp_hostfwd: dlsym(slirp_new) failed: %s\n", dlerror());
        return NULL;
    }

    Slirp *slirp = real_slirp_new(cfg, callbacks, opaque);
    if (!slirp) return NULL;

    const char *host_port  = getenv("HARNESS_HOST_PORT");
    const char *guest_port = getenv("HARNESS_GUEST_PORT");
    const char *is_udp     = getenv("HARNESS_GUEST_UDP");
    if (!host_port || !guest_port) return slirp;

    struct in_addr host  = { .s_addr = htonl(INADDR_LOOPBACK) };
    struct in_addr guest = { .s_addr = 0 };
    inet_pton(AF_INET, "10.0.2.15", &guest);

    int udp = is_udp && is_udp[0] == '1';
    int rc = slirp_add_hostfwd(slirp, udp, host, atoi(host_port),
                               guest, atoi(guest_port));
    if (rc != 0) {
        fprintf(stderr, "harness: slirp_add_hostfwd(%s/%s → guest:%s) failed\n",
                udp ? "udp" : "tcp", host_port, guest_port);
    } else {
        fprintf(stderr, "harness: hostfwd %s 127.0.0.1:%s → 10.0.2.15:%s\n",
                udp ? "udp" : "tcp", host_port, guest_port);
    }
    return slirp;
}
