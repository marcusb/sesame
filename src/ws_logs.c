#include "ws_logs.h"

#include <stdio.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/http/server.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/websocket.h>

#include "log_ring_buf.h"

LOG_MODULE_REGISTER(ws_logs, LOG_LEVEL_INF);

#define MAX_WS_CLIENTS 2

static int s_client_fds[MAX_WS_CLIENTS] = {-1, -1};
static struct k_mutex s_client_lock;
static struct k_sem s_client_sem;
static bool s_initialized;

static int backlog_chunk_cb(const char* chunk, size_t len, void* ctx) {
    int fd = (int)(intptr_t)ctx;
    return zsock_send(fd, chunk, len, 0);
}

static int ws_logs_setup(int ws_sock, struct http_request_ctx* request_ctx,
                         void* user_data) {
    ARG_UNUSED(request_ctx);
    ARG_UNUSED(user_data);

    if (ws_sock < 0) {
        return -EINVAL;
    }

    LOG_INF("WS log client connecting on socket %d", ws_sock);

    /* Replay initial backlog from the 16KB SRAM1 ring buffer */
    log_ring_buf_read_chunks(backlog_chunk_cb, (void*)(intptr_t)ws_sock);

    k_mutex_lock(&s_client_lock, K_FOREVER);
    int slot = -1;
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (s_client_fds[i] < 0) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        zsock_close(s_client_fds[0]);
        s_client_fds[0] = ws_sock;
    } else {
        s_client_fds[slot] = ws_sock;
    }
    k_mutex_unlock(&s_client_lock);

    /* Wake up monitor thread */
    k_sem_give(&s_client_sem);

    return 0;
}

static uint8_t s_ws_recv_buffer[256];

struct http_resource_detail_websocket ws_logs_resource_detail = {
    .common =
        {
            .type = HTTP_RESOURCE_TYPE_WEBSOCKET,
            .bitmask_of_supported_http_methods = BIT(HTTP_GET),
        },
    .cb = ws_logs_setup,
    .data_buffer = s_ws_recv_buffer,
    .data_buffer_len = sizeof(s_ws_recv_buffer),
    .user_data = NULL,
};

static char s_broadcast_buf[256];
static size_t s_broadcast_len = 0;

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

    bool has_any = false;
    for (int i = 0; i < MAX_WS_CLIENTS; i++) {
        if (s_client_fds[i] >= 0) {
            has_any = true;
            break;
        }
    }
    if (!has_any) {
        s_broadcast_len = 0;
        k_mutex_unlock(&s_client_lock);
        return;
    }

    for (size_t i = 0; i < len; i++) {
        s_broadcast_buf[s_broadcast_len++] = data[i];
        if (data[i] == '\n' || s_broadcast_len >= sizeof(s_broadcast_buf)) {
            for (int c = 0; c < MAX_WS_CLIENTS; c++) {
                int fd = s_client_fds[c];
                if (fd >= 0) {
                    int ret = zsock_send(fd, s_broadcast_buf, s_broadcast_len,
                                         ZSOCK_MSG_DONTWAIT);
                    if (ret < 0 && ret != -EAGAIN && ret != -EWOULDBLOCK) {
                        zsock_close(fd);
                        s_client_fds[c] = -1;
                    }
                }
            }
            s_broadcast_len = 0;
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

static void ws_logs_task(void* p1, void* p2, void* p3) {
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);

    k_mutex_init(&s_client_lock);
    k_sem_init(&s_client_sem, 0, 1);
    s_initialized = true;

    while (1) {
        k_mutex_lock(&s_client_lock, K_FOREVER);
        struct zsock_pollfd pfds[MAX_WS_CLIENTS];
        int client_slots[MAX_WS_CLIENTS];
        int num_pfds = 0;
        for (int i = 0; i < MAX_WS_CLIENTS; i++) {
            if (s_client_fds[i] >= 0) {
                pfds[num_pfds].fd = s_client_fds[i];
                pfds[num_pfds].events = ZSOCK_POLLIN;
                pfds[num_pfds].revents = 0;
                client_slots[num_pfds] = i;
                num_pfds++;
            }
        }
        k_mutex_unlock(&s_client_lock);

        if (num_pfds == 0) {
            k_sem_take(&s_client_sem, K_FOREVER);
            continue;
        }

        int poll_ret = zsock_poll(pfds, num_pfds, 1000);
        if (poll_ret <= 0) {
            continue;
        }

        k_mutex_lock(&s_client_lock, K_FOREVER);
        for (int i = 0; i < num_pfds; i++) {
            int slot = client_slots[i];
            if (slot < 0 || s_client_fds[slot] != pfds[i].fd) {
                continue;
            }
            if (pfds[i].revents &
                (ZSOCK_POLLIN | ZSOCK_POLLHUP | ZSOCK_POLLERR)) {
                uint8_t dummy[32];
                ssize_t n = zsock_recv(s_client_fds[slot], dummy, sizeof(dummy),
                                       ZSOCK_MSG_DONTWAIT);
                if (n <= 0) {
                    LOG_INF("WS client %d disconnected", s_client_fds[slot]);
                    zsock_close(s_client_fds[slot]);
                    s_client_fds[slot] = -1;
                }
            }
        }
        k_mutex_unlock(&s_client_lock);
    }
}

void ws_logs_init(void) { /* Thread is auto-started by K_THREAD_DEFINE */ }

K_THREAD_DEFINE(ws_logs_tid, 2048, ws_logs_task, NULL, NULL, NULL, 7, 0, 0);
