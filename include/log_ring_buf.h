#pragma once

#include <stddef.h>
#include <stdint.h>

#define LOG_RING_BUF_SIZE (16 * 1024)

#ifdef __cplusplus
extern "C" {
#endif

struct log_chunks {
    const char* chunk1;
    size_t len1;
    const char* chunk2;
    size_t len2;
};

typedef int (*log_ring_buf_chunk_cb)(const char* chunk, size_t len, void* ctx);

void log_ring_buf_init(void);
void log_ring_buf_write(const char* data, size_t len);
void log_ring_buf_get_chunks(struct log_chunks* chunks);
int log_ring_buf_read_chunks(log_ring_buf_chunk_cb cb, void* ctx);
size_t log_ring_buf_get_count(void);
void log_ring_buf_clear(void);

#ifdef __cplusplus
}
#endif
