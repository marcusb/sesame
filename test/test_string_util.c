#include <errno.h>
#include <string.h>

#include "string_util.h"
#include "unity.h"

static void test_strtcpy_zero_dsize(void) {
    char buf[4] = "xyz";
    errno = 0;
    ssize_t r = strtcpy(buf, "hello", 0);
    TEST_ASSERT_EQUAL(-1, r);
    TEST_ASSERT_EQUAL(ENOBUFS, errno);
}

static void test_strtcpy_normal(void) {
    char buf[16];
    ssize_t r = strtcpy(buf, "hello", sizeof(buf));
    TEST_ASSERT_EQUAL(5, r);
    TEST_ASSERT_EQUAL_STRING("hello", buf);
}

static void test_strtcpy_exact_fit(void) {
    char buf[6];
    ssize_t r = strtcpy(buf, "hello", sizeof(buf));
    TEST_ASSERT_EQUAL(5, r);
    TEST_ASSERT_EQUAL_STRING("hello", buf);
    TEST_ASSERT_EQUAL('\0', buf[5]);
}

static void test_strtcpy_truncation(void) {
    char buf[4];
    errno = 0;
    ssize_t r = strtcpy(buf, "hello", sizeof(buf));
    TEST_ASSERT_EQUAL(-1, r);
    TEST_ASSERT_EQUAL(E2BIG, errno);
    TEST_ASSERT_EQUAL_STRING("hel", buf);
    TEST_ASSERT_EQUAL('\0', buf[3]);
}

static void test_stpecpy_null_propagation(void) {
    char buf[8];
    char* end = buf + sizeof(buf);
    char* r = stpecpy(NULL, end, "hello");
    TEST_ASSERT_NULL(r);
}

static void test_stpecpy_chained(void) {
    char buf[16];
    char* end = buf + sizeof(buf);
    char* p = stpecpy(buf, end, "foo");
    TEST_ASSERT_NOT_NULL(p);
    p = stpecpy(p, end, "bar");
    TEST_ASSERT_NOT_NULL(p);
    TEST_ASSERT_EQUAL_STRING("foobar", buf);
}

static void test_stpecpy_truncation(void) {
    char buf[5];
    char* end = buf + sizeof(buf);
    char* r = stpecpy(buf, end, "hello!");
    TEST_ASSERT_NULL(r);
    TEST_ASSERT_EQUAL_STRING("hell", buf);
}

void run_test_string_util(void) {
    UnitySetTestFile(__FILE__);
    RUN_TEST(test_strtcpy_zero_dsize);
    RUN_TEST(test_strtcpy_normal);
    RUN_TEST(test_strtcpy_exact_fit);
    RUN_TEST(test_strtcpy_truncation);
    RUN_TEST(test_stpecpy_null_propagation);
    RUN_TEST(test_stpecpy_chained);
    RUN_TEST(test_stpecpy_truncation);
}
