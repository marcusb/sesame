#include "ws_logs.h"

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/base64.h>

#include "log_ring_buf.h"
#include "network.h"

LOG_MODULE_REGISTER(ws_logs, LOG_LEVEL_INF);

#define WS_MAGIC "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
#define MAX_WS_CLIENTS 2
#define WS_REQ_BUF_SIZE 1024

static int s_client_fds[MAX_WS_CLIENTS] = {-1, -1};
static struct k_mutex s_client_lock;
static bool s_initialized;

/* --- Self-contained RFC 3174 SHA-1 implementation --- */

struct sha1_ctx {
    uint32_t state[5];
    uint32_t count[2];
    uint8_t buffer[64];
};

#define SHA1_ROL(value, bits) (((value) << (bits)) | ((value) >> (32 - (bits))))

static void sha1_transform(uint32_t state[5], const uint8_t buffer[64]) {
    uint32_t a = state[0], b = state[1], c = state[2], d = state[3],
             e = state[4];
    uint32_t w[80];

    for (int i = 0; i < 16; i++) {
        w[i] = ((uint32_t)buffer[i * 4] << 24) |
               ((uint32_t)buffer[i * 4 + 1] << 16) |
               ((uint32_t)buffer[i * 4 + 2] << 8) |
               ((uint32_t)buffer[i * 4 + 3]);
    }
    for (int i = 16; i < 80; i++) {
        w[i] = SHA1_ROL(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
    }

    for (int i = 0; i < 80; i++) {
        uint32_t f, k;
        if (i < 20) {
            f = (b & c) | ((~b) & d);
            k = 0x5A827999;
        } else if (i < 40) {
            f = b ^ c ^ d;
            k = 0x6ED9EBA1;
        } else if (i < 60) {
            f = (b & c) | (b & d) | (c & d);
            k = 0x8F1BBCDC;
        } else {
            f = b ^ c ^ d;
            k = 0xCA62C1D6;
        }
        uint32_t temp = SHA1_ROL(a, 5) + f + e + k + w[i];
        e = d;
        d = c;
        c = SHA1_ROL(b, 30);
        b = a;
        a = temp;
    }

    state[0] += a;
    state[1] += b;
    state[2] += c;
    state[3] += d;
    state[4] += e;
}

static void sha1_init(struct sha1_ctx* ctx) {
    ctx->state[0] = 0x67452301;
    ctx->state[1] = 0xEFCDAB89;
    ctx->state[2] = 0x98BADCFE;
    ctx->state[3] = 0x10325476;
    ctx->state[4] = 0xC3D2E1F0;
    ctx->count[0] = ctx->count[1] = 0;
}

static void sha1_update(struct sha1_ctx* ctx, const uint8_t* data, size_t len) {
    size_t i = 0;
    size_t j = (ctx->count[0] >> 3) & 63;
    if ((ctx->count[0] += (uint32_t)(len << 3)) < (len << 3)) {
        ctx->count[1]++;
    }
    ctx->count[1] += (uint32_t)(len >> 29);
    if ((j + len) > 63) {
        memcpy(&ctx->buffer[j], data, (i = 64 - j));
        sha1_transform(ctx->state, ctx->buffer);
        for (; i + 63 < len; i += 64) {
            sha1_transform(ctx->state, &data[i]);
        }
        j = 0;
    }
    memcpy(&ctx->buffer[j], &data[i], len - i);
}

static void sha1_final(struct sha1_ctx* ctx, uint8_t digest[20]) {
    uint8_t finalcount[8];
    for (int i = 0; i < 8; i++) {
        finalcount[i] =
            (uint8_t)((ctx->count[(i >= 4 ? 0 : 1)] >> ((3 - (i & 3)) * 8)) &
                      255);
    }
    uint8_t c = 0200;
    sha1_update(ctx, &c, 1);
    while ((ctx->count[0] & 504) != 448) {
        c = 0000;
        sha1_update(ctx, &c, 1);
    }
    sha1_update(ctx, finalcount, 8);
    for (int i = 0; i < 20; i++) {
        digest[i] =
            (uint8_t)((ctx->state[i >> 2] >> ((3 - (i & 3)) * 8)) & 255);
    }
}

/* --- WebSocket Framing & Sending --- */

static int send_ws_frame(int fd, uint8_t opcode, const uint8_t* payload,
                         size_t len) {
    if (fd < 0) {
        return -EBADF;
    }

    uint8_t hdr[10];
    size_t hdr_len = 0;
    hdr[0] = 0x80 | (opcode & 0x0F); /* FIN + opcode */

    if (len < 126) {
        hdr[1] = (uint8_t)len;
        hdr_len = 2;
    } else if (len <= 65535) {
        hdr[1] = 126;
        hdr[2] = (uint8_t)((len >> 8) & 0xFF);
        hdr[3] = (uint8_t)(len & 0xFF);
        hdr_len = 4;
    } else {
        hdr[1] = 127;
        for (int i = 7; i >= 0; i--) {
            hdr[2 + (7 - i)] = (uint8_t)((len >> (i * 8)) & 0xFF);
        }
        hdr_len = 10;
    }

    int ret = zsock_send(fd, hdr, hdr_len, ZSOCK_MSG_DONTWAIT);
    if (ret < 0) {
        return ret;
    }

    size_t sent = 0;
    while (sent < len) {
        ret = zsock_send(fd, payload + sent, len - sent, ZSOCK_MSG_DONTWAIT);
        if (ret < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break;
            }
            return ret;
        }
        sent += ret;
    }
    return 0;
}

static int backlog_chunk_cb(const char* chunk, size_t len, void* ctx) {
    int fd = (int)(intptr_t)ctx;
    return send_ws_frame(fd, 0x1 /* text */, (const uint8_t*)chunk, len);
}

void ws_logs_broadcast(const char* data, size_t len) {
    if (!data || len == 0 || !s_initialized) {
        return;
    }

    if (k_is_in_isr()) {
        return;
    }

    if (k_mutex_lock(&s_client_lock, K_MSEC(20)) != 0) {
        return;
    }

    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        int fd = s_client_fds[i];
        if (fd >= 0) {
            int ret =
                send_ws_frame(fd, 0x1 /* text */, (const uint8_t*)data, len);
            if (ret < 0 && ret != -EAGAIN && ret != -EWOULDBLOCK) {
                LOG_INF("WS client %d disconnected on send (%d)", fd, ret);
                zsock_close(fd);
                s_client_fds[i] = -1;
            }
        }
    }

    k_mutex_unlock(&s_client_lock);
}

bool ws_logs_has_clients(void) {
    if (!s_initialized) {
        return false;
    }
    bool has_any = false;
    if (k_mutex_lock(&s_client_lock, K_MSEC(20)) == 0) {
        for (int i = 0; i < MAX_WS_CLIENTS; i++) {
            if (s_client_fds[i] >= 0) {
                has_any = true;
                break;
            }
        }
        k_mutex_unlock(&s_client_lock);
    }
    return has_any;
}

static const char b64_table[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

static void base64_encode_sha1(const uint8_t in[20], char out[29]) {
    size_t i = 0, j = 0;
    while (i < 18) {
        uint32_t triple =
            ((uint32_t)in[i] << 16) | ((uint32_t)in[i + 1] << 8) | in[i + 2];
        out[j++] = b64_table[(triple >> 18) & 0x3F];
        out[j++] = b64_table[(triple >> 12) & 0x3F];
        out[j++] = b64_table[(triple >> 6) & 0x3F];
        out[j++] = b64_table[triple & 0x3F];
        i += 3;
    }
    uint32_t triple = ((uint32_t)in[18] << 16) | ((uint32_t)in[19] << 8);
    out[j++] = b64_table[(triple >> 18) & 0x3F];
    out[j++] = b64_table[(triple >> 12) & 0x3F];
    out[j++] = b64_table[(triple >> 6) & 0x3F];
    out[j++] = '=';
    out[j] = '\0';
}

/* --- WebSocket Handshake & Server Task --- */

static int handle_handshake(int client_fd) {
    char buf[WS_REQ_BUF_SIZE];
    ssize_t n = zsock_recv(client_fd, buf, sizeof(buf) - 1, 0);
    if (n <= 0) {
        return -1;
    }
    buf[n] = '\0';

    if (strncmp(buf, "GET ", 4) != 0) {
        return -1;
    }

    char* key_hdr = strstr(buf, "Sec-WebSocket-Key: ");
    if (!key_hdr) {
        key_hdr = strstr(buf, "sec-websocket-key: ");
    }
    if (!key_hdr) {
        return -1;
    }

    key_hdr += 19;
    char key[64] = {0};
    int klen = 0;
    while (*key_hdr && *key_hdr != '\r' && *key_hdr != '\n' &&
           klen < sizeof(key) - 1) {
        key[klen++] = *key_hdr++;
    }
    key[klen] = '\0';

    /* Concatenate key + WS_MAGIC */
    char combined[128];
    snprintf(combined, sizeof(combined), "%s%s", key, WS_MAGIC);

    /* Compute SHA-1 */
    struct sha1_ctx ctx;
    uint8_t digest[20];
    sha1_init(&ctx);
    sha1_update(&ctx, (const uint8_t*)combined, strlen(combined));
    sha1_final(&ctx, digest);

    /* Base64 encode */
    char accept_key[32];
    base64_encode_sha1(digest, accept_key);

    /* Send HTTP 101 Response */
    char resp[256];
    int rlen = snprintf(resp, sizeof(resp),
                        "HTTP/1.1 101 Switching Protocols\r\n"
                        "Upgrade: websocket\r\n"
                        "Connection: Upgrade\r\n"
                        "Sec-WebSocket-Accept: %s\r\n\r\n",
                        accept_key);
    if (zsock_send(client_fd, resp, rlen, 0) < 0) {
        return -1;
    }

    LOG_INF("WS client accepted on socket %d", client_fd);

    /* Replay initial backlog from the 16KB SRAM1 ring buffer */
    log_ring_buf_read_chunks(backlog_chunk_cb, (void*)(intptr_t)client_fd);

    return 0;
}

static void ws_logs_task(void* p1, void* p2, void* p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    k_mutex_init(&s_client_lock);
    s_initialized = true;

    /* Wait for network to be ready */
    while (!network_is_up()) {
        k_sleep(K_MSEC(500));
    }

    int listen_fd = zsock_socket(AF_INET6, SOCK_STREAM, IPPROTO_TCP);
    if (listen_fd < 0) {
        LOG_ERR("Failed to create WS listen socket: %d", errno);
        return;
    }

    int opt = 1;
    zsock_setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in6 addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin6_family = AF_INET6;
    addr.sin6_port = htons(WS_LOGS_PORT);
    addr.sin6_addr = in6addr_any;

    if (zsock_bind(listen_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        LOG_ERR("Failed to bind WS port %d: %d", WS_LOGS_PORT, errno);
        zsock_close(listen_fd);
        return;
    }

    if (zsock_listen(listen_fd, 2) < 0) {
        LOG_ERR("Failed to listen on WS port %d: %d", WS_LOGS_PORT, errno);
        zsock_close(listen_fd);
        return;
    }

    LOG_INF("WebSocket log streaming listening on port %d", WS_LOGS_PORT);

    while (1) {
        struct zsock_pollfd pfds[1 + MAX_WS_CLIENTS];
        int num_pfds = 0;

        pfds[num_pfds].fd = listen_fd;
        pfds[num_pfds].events = ZSOCK_POLLIN;
        pfds[num_pfds].revents = 0;
        num_pfds++;

        k_mutex_lock(&s_client_lock, K_FOREVER);
        int client_pfd_idx[MAX_WS_CLIENTS];
        for (int i = 0; i < MAX_WS_CLIENTS; i++) {
            client_pfd_idx[i] = -1;
            if (s_client_fds[i] >= 0) {
                client_pfd_idx[i] = num_pfds;
                pfds[num_pfds].fd = s_client_fds[i];
                pfds[num_pfds].events = ZSOCK_POLLIN;
                pfds[num_pfds].revents = 0;
                num_pfds++;
            }
        }
        k_mutex_unlock(&s_client_lock);

        int poll_ret = zsock_poll(pfds, num_pfds, 1000);
        if (poll_ret <= 0) {
            continue;
        }

        /* Check listening socket */
        if (pfds[0].revents & ZSOCK_POLLIN) {
            struct sockaddr_in6 client_addr;
            socklen_t addr_len = sizeof(client_addr);
            int new_fd = zsock_accept(listen_fd, (struct sockaddr*)&client_addr,
                                      &addr_len);
            if (new_fd >= 0) {
                struct timeval tv = {.tv_sec = 2, .tv_usec = 0};
                zsock_setsockopt(new_fd, SOL_SOCKET, SO_RCVTIMEO, &tv,
                                 sizeof(tv));

                if (handle_handshake(new_fd) == 0) {
                    /* Handshake succeeded, add to client list */
                    tv.tv_sec = 0;
                    tv.tv_usec = 0;
                    zsock_setsockopt(new_fd, SOL_SOCKET, SO_RCVTIMEO, &tv,
                                     sizeof(tv));

                    k_mutex_lock(&s_client_lock, K_FOREVER);
                    bool added = false;
                    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
                        if (s_client_fds[i] < 0) {
                            s_client_fds[i] = new_fd;
                            added = true;
                            break;
                        }
                    }
                    if (!added) {
                        /* Too many clients, close the oldest slot 0 and replace
                         */
                        zsock_close(s_client_fds[0]);
                        s_client_fds[0] = new_fd;
                    }
                    k_mutex_unlock(&s_client_lock);
                } else {
                    zsock_close(new_fd);
                }
            }
        }

        /* Check active clients for incoming frames (ping, close) or disconnect
         */
        k_mutex_lock(&s_client_lock, K_FOREVER);
        for (int i = 0; i < MAX_WS_CLIENTS; i++) {
            int pidx = client_pfd_idx[i];
            if (pidx < 0) {
                continue;
            }
            if (pfds[pidx].revents &
                (ZSOCK_POLLIN | ZSOCK_POLLHUP | ZSOCK_POLLERR)) {
                uint8_t frame_hdr[2];
                ssize_t n = zsock_recv(s_client_fds[i], frame_hdr,
                                       sizeof(frame_hdr), ZSOCK_MSG_DONTWAIT);
                if (n <= 0) {
                    LOG_INF("WS client %d disconnected", s_client_fds[i]);
                    zsock_close(s_client_fds[i]);
                    s_client_fds[i] = -1;
                } else {
                    uint8_t opcode = frame_hdr[0] & 0x0F;
                    if (opcode == 0x8) {
                        /* Close frame */
                        LOG_INF("WS client %d sent close frame",
                                s_client_fds[i]);
                        zsock_close(s_client_fds[i]);
                        s_client_fds[i] = -1;
                    } else if (opcode == 0x9) {
                        /* Ping frame -> send Pong */
                        uint8_t pong[2] = {0x8A, 0x00};
                        zsock_send(s_client_fds[i], pong, sizeof(pong),
                                   ZSOCK_MSG_DONTWAIT);
                    }
                }
            }
        }
        k_mutex_unlock(&s_client_lock);
    }
}

void ws_logs_init(void) { /* Thread is auto-started by K_THREAD_DEFINE */ }

K_THREAD_DEFINE(ws_logs_tid, 2048, ws_logs_task, NULL, NULL, NULL, 7, 0, 0);
