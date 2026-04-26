#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "berry.h"
#include "matter_test_utils.h"
#include "psm-v2.h"
#include "task.h"
#include "unity.h"

extern void stub_psm_set_data(const uint8_t* data, size_t len);
extern void stub_psm_get_written(const uint8_t** out, size_t* out_len);
extern void stub_psm_set_error(int err);

extern psm_hnd_t psm_hnd;

void setUp(void) {
    matter_test_setup();
    stub_psm_set_data(NULL, 0);
}

void tearDown(void) { matter_test_teardown(); }

void test_tasmota_config(void) {
    be_assert_success("import tasmota; tasmota.set_config('k', 'v')");
    const uint8_t* data;
    size_t len;
    stub_psm_get_written(&data, &len);
    TEST_ASSERT_EQUAL(1, len);
    be_assert_success("assert(tasmota.get_config('k') == 'v')");
}

void test_persist(void) {
    be_assert_success("import persist; persist.setmember('f', 'b')");
    be_assert_success("assert(persist.find('f') == 'b')");
}

void run_tests(void) {
    UnitySetTestFile(__FILE__);
    RUN_TEST(test_tasmota_config);
    RUN_TEST(test_persist);
}
