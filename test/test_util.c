#include <stdint.h>
#include <string.h>

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "test_capture.h"
#include "unity.h"
#include "util.h"

static void wait_for_log(void) { vTaskDelay(pdMS_TO_TICKS(50)); }

static void test_hexdump_zero_len(void) {
    capture_drain_and_reset();
    debug_hexdump('T', (const uint8_t*)"\xAB", 0);
    wait_for_log();
    TEST_ASSERT_EQUAL(0, capture_count);
}

static void test_hexdump_printable_byte(void) {
    uint8_t data[] = {0x41}; /* 'A' */
    capture_drain_and_reset();
    debug_hexdump('R', data, sizeof(data));
    wait_for_log();
    TEST_ASSERT_GREATER_THAN(0, capture_count);
    TEST_ASSERT_NOT_NULL(strstr(capture_msgs[0], "41"));
    TEST_ASSERT_NOT_NULL(strstr(capture_msgs[0], "A"));
}

static void test_hexdump_nonprintable_byte(void) {
    uint8_t data[] = {0x01};
    capture_drain_and_reset();
    debug_hexdump('T', data, sizeof(data));
    wait_for_log();
    TEST_ASSERT_GREATER_THAN(0, capture_count);
    TEST_ASSERT_NOT_NULL(strstr(capture_msgs[0], "01"));
    TEST_ASSERT_NOT_NULL(strstr(capture_msgs[0], "."));
}

static void test_hexdump_16_bytes(void) {
    uint8_t data[16];
    for (int i = 0; i < 16; i++) data[i] = (uint8_t)i;
    capture_drain_and_reset();
    debug_hexdump('R', data, sizeof(data));
    wait_for_log();
    /* Exactly one log line for 16 bytes (boundary fires at i==15) */
    TEST_ASSERT_EQUAL(1, capture_count);
    TEST_ASSERT_NOT_NULL(strstr(capture_msgs[0], "00"));
}

static void test_hexdump_17_bytes(void) {
    uint8_t data[17];
    for (int i = 0; i < 17; i++) data[i] = (uint8_t)i;
    capture_drain_and_reset();
    debug_hexdump('R', data, sizeof(data));
    wait_for_log();
    /* Two log lines: first full line at i==15, second for byte 16 */
    TEST_ASSERT_EQUAL(2, capture_count);
}

void run_tests_util(void) {
    UnitySetTestFile(__FILE__);
    RUN_TEST(test_hexdump_zero_len);
    RUN_TEST(test_hexdump_printable_byte);
    RUN_TEST(test_hexdump_nonprintable_byte);
    RUN_TEST(test_hexdump_16_bytes);
    RUN_TEST(test_hexdump_17_bytes);
}
