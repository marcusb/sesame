#include <string.h>

/* FreeRTOS */
#include "FreeRTOS.h"
#include "logging.h"
#include "logging_levels.h"
#include "task.h"
#include "test_capture.h"
#include "unity.h"

static void dummy_backend(const log_msg_t* log) { (void)log; }

/*
 * These tests run in order and share persistent logging state:
 *   - capture_backend was registered by test_runner before UNITY_BEGIN()
 *   - test_register_second registers dummy_backend in the second slot
 *   - test_register_full verifies no third slot is available
 */

static void test_register_second_backend(void) {
    int r = register_log_backend(dummy_backend);
    TEST_ASSERT_EQUAL(0, r);
}

static void test_register_backend_full(void) {
    int r = register_log_backend(dummy_backend);
    TEST_ASSERT_EQUAL(-1, r);
}

static void test_log_message_delivered(void) {
    capture_drain_and_reset();
    vLoggingPrintfInfo("test message %d", 42);
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_GREATER_THAN(0, capture_count);
    TEST_ASSERT_NOT_NULL(strstr(capture_msgs[0], "test message 42"));
    TEST_ASSERT_EQUAL(LOG_INFO, capture_levels[0]);
}

static void test_log_file_line_prefix(void) {
    capture_drain_and_reset();
    vLoggingPrintfWithFileAndLine("foo.c", 42, "prefixed");
    vTaskDelay(pdMS_TO_TICKS(50));
    TEST_ASSERT_GREATER_THAN(0, capture_count);
    TEST_ASSERT_NOT_NULL(strstr(capture_msgs[0], "[foo.c:42]"));
    TEST_ASSERT_NOT_NULL(strstr(capture_msgs[0], "prefixed"));
}

void run_test_logging(void) {
    UnitySetTestFile(__FILE__);
    RUN_TEST(test_register_second_backend);
    RUN_TEST(test_register_backend_full);
    RUN_TEST(test_log_message_delivered);
    RUN_TEST(test_log_file_line_prefix);
}
