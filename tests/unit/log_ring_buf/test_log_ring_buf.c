#include <errno.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/ztest.h>

#include "log_ring_buf.h"
#include "ws_logs.h"

LOG_MODULE_REGISTER(test_log_ring_buf, LOG_LEVEL_DBG);

static char s_last_broadcast[1024];
static size_t s_last_broadcast_len;
static int s_broadcast_call_count;

void ws_logs_broadcast(const char* data, size_t len) {
    s_broadcast_call_count++;
    if (len < sizeof(s_last_broadcast)) {
        memcpy(s_last_broadcast, data, len);
        s_last_broadcast[len] = '\0';
        s_last_broadcast_len = len;
    } else {
        memcpy(s_last_broadcast, data, sizeof(s_last_broadcast) - 1);
        s_last_broadcast[sizeof(s_last_broadcast) - 1] = '\0';
        s_last_broadcast_len = sizeof(s_last_broadcast) - 1;
    }
}

bool ws_logs_has_clients(void) { return false; }

static void before_each(void* fixture) {
    ARG_UNUSED(fixture);
    log_ring_buf_init();
    log_ring_buf_clear();
    s_broadcast_call_count = 0;
    s_last_broadcast_len = 0;
    memset(s_last_broadcast, 0, sizeof(s_last_broadcast));
}

ZTEST_SUITE(log_ring_buf, NULL, NULL, before_each, NULL, NULL);

ZTEST(log_ring_buf, test_init_and_clear) {
    zassert_equal(log_ring_buf_get_count(), 0,
                  "Buffer should be empty on init");

    struct log_chunks chunks;
    log_ring_buf_get_chunks(&chunks);
    zassert_equal(chunks.len1, 0, "Chunk 1 should have len 0");
    zassert_equal(chunks.len2, 0, "Chunk 2 should have len 0");

    const char* sample = "Testing clear";
    log_ring_buf_write(sample, strlen(sample));
    zassert_equal(log_ring_buf_get_count(), strlen(sample),
                  "Count should match write");

    log_ring_buf_clear();
    zassert_equal(log_ring_buf_get_count(), 0, "Count should be 0 after clear");

    log_ring_buf_get_chunks(&chunks);
    zassert_equal(chunks.len1, 0, "Chunk 1 should be empty after clear");
    zassert_equal(chunks.len2, 0, "Chunk 2 should be empty after clear");
}

ZTEST(log_ring_buf, test_write_single_and_chunks) {
    const char* msg = "Hello from Sesame log buffer test!";
    size_t len = strlen(msg);

    log_ring_buf_write(msg, len);

    zassert_equal(log_ring_buf_get_count(), len, "Count mismatch");

    struct log_chunks chunks;
    log_ring_buf_get_chunks(&chunks);
    zassert_equal(chunks.len1, len, "Chunk 1 length mismatch");
    zassert_equal(chunks.len2, 0, "Chunk 2 should be 0 for unwrapped buffer");
    zassert_not_null(chunks.chunk1, "Chunk 1 pointer should not be null");
    zassert_equal(memcmp(chunks.chunk1, msg, len), 0,
                  "Buffer content mismatch");

    zassert_equal(s_broadcast_call_count, 1,
                  "Broadcast callback should be called once");
    zassert_equal(s_last_broadcast_len, len, "Broadcast len mismatch");
    zassert_equal(strcmp(s_last_broadcast, msg), 0,
                  "Broadcast content mismatch");
}

struct chunk_collect_ctx {
    char buf[512];
    size_t len;
    int calls;
};

static int chunk_collector_cb(const char* chunk, size_t len, void* ctx) {
    struct chunk_collect_ctx* c = (struct chunk_collect_ctx*)ctx;
    c->calls++;
    if (c->len + len < sizeof(c->buf)) {
        memcpy(c->buf + c->len, chunk, len);
        c->len += len;
        c->buf[c->len] = '\0';
    }
    return 0;
}

ZTEST(log_ring_buf, test_read_chunks_callback) {
    const char* line1 = "First log line\n";
    const char* line2 = "Second log line\n";

    log_ring_buf_write(line1, strlen(line1));
    log_ring_buf_write(line2, strlen(line2));

    struct chunk_collect_ctx ctx;
    memset(&ctx, 0, sizeof(ctx));

    int ret = log_ring_buf_read_chunks(chunk_collector_cb, &ctx);
    zassert_equal(ret, 0, "log_ring_buf_read_chunks returned error: %d", ret);
    zassert_equal(ctx.calls, 1, "Unwrapped buffer should invoke callback once");
    zassert_equal(ctx.len, strlen(line1) + strlen(line2),
                  "Total collected len mismatch");

    char expected[128];
    snprintf(expected, sizeof(expected), "%s%s", line1, line2);
    zassert_equal(strcmp(ctx.buf, expected), 0, "Collected content mismatch");

    /* Null callback error check */
    ret = log_ring_buf_read_chunks(NULL, &ctx);
    zassert_equal(ret, -EINVAL, "Expected -EINVAL for null callback");
}

ZTEST(log_ring_buf, test_null_and_zero_handling) {
    log_ring_buf_write(NULL, 100);
    zassert_equal(log_ring_buf_get_count(), 0,
                  "Write with NULL data should be ignored");

    log_ring_buf_write("test", 0);
    zassert_equal(log_ring_buf_get_count(), 0,
                  "Write with 0 len should be ignored");

    /* Should not crash with NULL chunks */
    log_ring_buf_get_chunks(NULL);
}

static uint8_t s_large_buf[10000];
static uint8_t s_reconstructed[LOG_RING_BUF_SIZE];

struct large_collect_ctx {
    size_t offset;
    int calls;
};

static int large_chunk_cb(const char* chunk, size_t len, void* ctx) {
    struct large_collect_ctx* c = (struct large_collect_ctx*)ctx;
    c->calls++;
    if (c->offset + len <= sizeof(s_reconstructed)) {
        memcpy(s_reconstructed + c->offset, chunk, len);
        c->offset += len;
    }
    return 0;
}

ZTEST(log_ring_buf, test_buffer_wrapping) {
    /* Fill buffer with 10,000 'A's, then 10,000 'B's (total 20,000 > 16,384) */
    memset(s_large_buf, 'A', sizeof(s_large_buf));
    log_ring_buf_write((const char*)s_large_buf, sizeof(s_large_buf));

    memset(s_large_buf, 'B', sizeof(s_large_buf));
    log_ring_buf_write((const char*)s_large_buf, sizeof(s_large_buf));

    zassert_equal(log_ring_buf_get_count(), LOG_RING_BUF_SIZE,
                  "Buffer count should cap at LOG_RING_BUF_SIZE");

    /* Verify log_ring_buf_get_chunks returns 2 chunks summing to
     * LOG_RING_BUF_SIZE */
    struct log_chunks chunks;
    log_ring_buf_get_chunks(&chunks);
    zassert_equal(chunks.len1 + chunks.len2, LOG_RING_BUF_SIZE,
                  "Sum of chunks should equal LOG_RING_BUF_SIZE");
    zassert_true(chunks.len1 > 0, "Chunk 1 should not be empty");
    zassert_true(chunks.len2 > 0, "Chunk 2 should not be empty");

    /* Verify chunk data order: oldest remaining data in chunk1, newest in
     * chunk2 */
    memcpy(s_reconstructed, chunks.chunk1, chunks.len1);
    memcpy(s_reconstructed + chunks.len1, chunks.chunk2, chunks.len2);

    /* 20,000 total written into 16,384 buffer:
     * Overwritten: 20000 - 16384 = 3616 bytes of 'A'.
     * Remaining 'A's: 10000 - 3616 = 6384 bytes.
     * Remaining 'B's: 10000 bytes.
     */
    const size_t expected_a = 20000 - 3616 - 10000; /* 6384 */
    for (size_t i = 0; i < expected_a; i++) {
        zassert_equal(s_reconstructed[i], 'A', "Byte %zu should be 'A'", i);
    }
    for (size_t i = expected_a; i < LOG_RING_BUF_SIZE; i++) {
        zassert_equal(s_reconstructed[i], 'B', "Byte %zu should be 'B'", i);
    }

    /* Verify log_ring_buf_read_chunks yields the same data across 2 callback
     * invocations */
    struct large_collect_ctx ctx;
    ctx.offset = 0;
    ctx.calls = 0;
    memset(s_reconstructed, 0, sizeof(s_reconstructed));

    int ret = log_ring_buf_read_chunks(large_chunk_cb, &ctx);
    zassert_equal(ret, 0, "read_chunks failed: %d", ret);
    zassert_equal(ctx.calls, 2, "Wrapped buffer should invoke callback twice");
    zassert_equal(ctx.offset, LOG_RING_BUF_SIZE, "Reconstructed size mismatch");

    for (size_t i = 0; i < expected_a; i++) {
        zassert_equal(s_reconstructed[i], 'A',
                      "Callback byte %zu should be 'A'", i);
    }
    for (size_t i = expected_a; i < LOG_RING_BUF_SIZE; i++) {
        zassert_equal(s_reconstructed[i], 'B',
                      "Callback byte %zu should be 'B'", i);
    }
}

ZTEST(log_ring_buf, test_exact_capacity_boundary) {
    /* Write exactly LOG_RING_BUF_SIZE bytes in chunks */
    const size_t half = LOG_RING_BUF_SIZE / 2;
    memset(s_large_buf, 'X', half);
    log_ring_buf_write((const char*)s_large_buf, half);
    log_ring_buf_write((const char*)s_large_buf, half);

    zassert_equal(log_ring_buf_get_count(), LOG_RING_BUF_SIZE,
                  "Should be full");

    struct log_chunks chunks;
    log_ring_buf_get_chunks(&chunks);
    zassert_equal(chunks.len1, LOG_RING_BUF_SIZE,
                  "Exactly full should have single chunk1");
    zassert_equal(chunks.len2, 0, "Chunk 2 should be 0 when s_head == 0");

    /* Write 1 additional byte to trigger wrap */
    const char extra = 'Z';
    log_ring_buf_write(&extra, 1);

    zassert_equal(log_ring_buf_get_count(), LOG_RING_BUF_SIZE,
                  "Count remains capped");
    log_ring_buf_get_chunks(&chunks);
    zassert_equal(chunks.len1, LOG_RING_BUF_SIZE - 1,
                  "len1 should be SIZE - 1");
    zassert_equal(chunks.len2, 1, "len2 should be 1");
    zassert_equal(chunks.chunk2[0], 'Z', "chunk2[0] should be new byte");
}

ZTEST(log_ring_buf, test_zephyr_log_backend_integration) {
    s_broadcast_call_count = 0;
    memset(s_last_broadcast, 0, sizeof(s_last_broadcast));

    /* Emit a Zephyr log message */
    LOG_INF("Zephyr log backend integration test token=987654");

    /* In immediate mode it's already processed, in deferred mode flush it */
    while (log_process()) {
    }

    zassert_true(log_ring_buf_get_count() > 0,
                 "Log message should be written to ring buffer");

    struct log_chunks chunks;
    log_ring_buf_get_chunks(&chunks);
    zassert_true(chunks.len1 > 0, "Chunk 1 should contain log output");

    /* Check that the emitted log token is present in the ring buffer */
    char temp[256];
    size_t copy_len =
        chunks.len1 < sizeof(temp) - 1 ? chunks.len1 : sizeof(temp) - 1;
    memcpy(temp, chunks.chunk1, copy_len);
    temp[copy_len] = '\0';

    zassert_not_null(strstr(temp, "token=987654"),
                     "Expected log string with token in ring buffer, got: %s",
                     temp);

    /* Also verify ws_logs_broadcast received the log line */
    zassert_true(s_broadcast_call_count > 0,
                 "ws_logs_broadcast should have been invoked");
}
