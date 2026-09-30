#include "log_ring_buf.h"

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log_backend.h>
#include <zephyr/logging/log_core.h>
#include <zephyr/logging/log_output.h>

#include "ws_logs.h"

/* Place the 16KB log ring buffer in SRAM1 */
static uint8_t s_log_buf[LOG_RING_BUF_SIZE]
    __attribute__((section(".sram1_log_buf")));

static size_t s_head;
static size_t s_count;
static struct k_mutex s_lock;
static bool s_initialized;

void log_ring_buf_init(void) {
    if (s_initialized) {
        return;
    }
    k_mutex_init(&s_lock);
    s_head = 0;
    s_count = 0;
    s_initialized = true;
}

void log_ring_buf_write(const char* data, size_t len) {
    if (!data || len == 0) {
        return;
    }

    if (!s_initialized) {
        log_ring_buf_init();
    }

    if (k_is_in_isr()) {
        /* Avoid mutex in ISR */
        return;
    }

    if (k_mutex_lock(&s_lock, K_MSEC(50)) == 0) {
        for (size_t i = 0; i < len; i++) {
            s_log_buf[s_head] = (uint8_t)data[i];
            s_head = (s_head + 1) % LOG_RING_BUF_SIZE;
            if (s_count < LOG_RING_BUF_SIZE) {
                s_count++;
            }
        }
        k_mutex_unlock(&s_lock);
    }

    /* Broadcast newly arrived log chunk to active WebSocket clients */
    ws_logs_broadcast(data, len);
}

void log_ring_buf_get_chunks(struct log_chunks* chunks) {
    if (!chunks) {
        return;
    }
    chunks->chunk1 = NULL;
    chunks->len1 = 0;
    chunks->chunk2 = NULL;
    chunks->len2 = 0;

    if (!s_initialized) {
        return;
    }

    if (k_mutex_lock(&s_lock, K_MSEC(100)) == 0) {
        if (s_count < LOG_RING_BUF_SIZE) {
            chunks->chunk1 = (const char*)s_log_buf;
            chunks->len1 = s_count;
        } else {
            chunks->chunk1 = (const char*)&s_log_buf[s_head];
            chunks->len1 = LOG_RING_BUF_SIZE - s_head;
            chunks->chunk2 = (const char*)&s_log_buf[0];
            chunks->len2 = s_head;
        }
        k_mutex_unlock(&s_lock);
    }
}

int log_ring_buf_read_chunks(log_ring_buf_chunk_cb cb, void* ctx) {
    if (!cb) {
        return -EINVAL;
    }

    if (!s_initialized) {
        return 0;
    }

    if (k_mutex_lock(&s_lock, K_MSEC(200)) != 0) {
        return -EBUSY;
    }

    int ret = 0;
    if (s_count < LOG_RING_BUF_SIZE) {
        if (s_count > 0) {
            ret = cb((const char*)s_log_buf, s_count, ctx);
        }
    } else {
        size_t tail_len = LOG_RING_BUF_SIZE - s_head;
        if (tail_len > 0) {
            ret = cb((const char*)&s_log_buf[s_head], tail_len, ctx);
        }
        if (ret == 0 && s_head > 0) {
            ret = cb((const char*)&s_log_buf[0], s_head, ctx);
        }
    }

    k_mutex_unlock(&s_lock);
    return ret;
}

size_t log_ring_buf_get_count(void) {
    if (!s_initialized) {
        return 0;
    }
    size_t count = 0;
    if (k_mutex_lock(&s_lock, K_MSEC(50)) == 0) {
        count = s_count;
        k_mutex_unlock(&s_lock);
    }
    return count;
}

void log_ring_buf_clear(void) {
    if (!s_initialized) {
        return;
    }
    if (k_mutex_lock(&s_lock, K_MSEC(50)) == 0) {
        s_head = 0;
        s_count = 0;
        k_mutex_unlock(&s_lock);
    }
}

/* --- Zephyr Log Backend Integration --- */

static int ring_buf_char_out(uint8_t* data, size_t length, void* ctx) {
    ARG_UNUSED(ctx);
    log_ring_buf_write((const char*)data, length);
    return length;
}

static uint8_t log_output_buf[128];
LOG_OUTPUT_DEFINE(log_output_ringbuf, ring_buf_char_out, log_output_buf,
                  sizeof(log_output_buf));

static void ringbuf_log_process(const struct log_backend* const backend,
                                union log_msg_generic* msg) {
    ARG_UNUSED(backend);
    uint32_t flags = LOG_OUTPUT_FLAG_FORMAT_SYSLOG | LOG_OUTPUT_FLAG_TIMESTAMP |
                     LOG_OUTPUT_FLAG_THREAD | LOG_OUTPUT_FLAG_CRLF_NONE;
    log_format_func_t log_output_func = log_format_func_t_get(LOG_OUTPUT_TEXT);
    log_output_func(&log_output_ringbuf, &msg->log, flags);
    log_output_flush(&log_output_ringbuf);
    ring_buf_char_out((uint8_t*)"\n", 1, NULL);
}

static void ringbuf_log_panic(const struct log_backend* const backend) {
    ARG_UNUSED(backend);
}

static void ringbuf_log_init(const struct log_backend* const backend) {
    ARG_UNUSED(backend);
    log_ring_buf_init();
}

static const struct log_backend_api log_backend_ringbuf_api = {
    .process = ringbuf_log_process,
    .panic = ringbuf_log_panic,
    .init = ringbuf_log_init,
};

LOG_BACKEND_DEFINE(log_backend_ringbuf, log_backend_ringbuf_api, true);
