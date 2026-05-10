#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "FreeRTOS.h"
#include "berry.h"
#include "matter_tasmota_shim.h"
#include "matter_test_utils.h"
#include "psm-v2.h"
#include "task.h"
#include "unity.h"

extern bvm* g_matter_vm;

extern void stub_psm_set_data(const uint8_t* data, size_t len);
extern void stub_psm_get_written(const uint8_t** out, size_t* out_len);
extern void stub_psm_set_error(int err);
extern void stub_psm_clear(void);

extern psm_hnd_t psm_hnd;

void setUp(void) {
    matter_test_setup();
    stub_psm_set_data(NULL, 0);
    be_dostring(g_matter_vm,
                "_matter_fast_cbs = []\n"
                "_matter_fast_cbs_once = []\n"
                "_matter_net_cbs = []\n"
                "_matter_drivers = []\n");
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

void test_tasmota_when_network_up_fires(void) {
    be_assert_success(
        "import tasmota; var info = tasmota.wifi(); assert(info != nil); "
        "assert(classname(info) == 'map')");
}

void test_tasmota_fast_loop_runs(void) {
    be_assert_success(
        "import tasmota\n"
        "_c = [0]\n"
        "def _tick() _c[0] += 1 end\n"
        "tasmota.add_fast_loop(_tick)\n");

    /* Simulate the application loop calling the tick function. */
    for (int i = 0; i < 5; i++) {
        matter_tasmota_tick(g_matter_vm);
    }

    be_assert_success("assert(_c[0] > 0)");
}

void test_persist(void) {
    be_assert_success("import persist; persist.setmember('f', 'b')");
    be_assert_success("assert(persist.find('f') == 'b')");
}

/* Regression: path.rename used to always return nil, which Berry treats as
 * falsy. Matter_Session_Store::save_fabrics() interprets a falsy return as
 * failure and logs "Saving Fabrics failed" + skips event_fabrics_saved(),
 * even though the fabric was persisted correctly. */
void test_path_rename_success_returns_true(void) {
    const uint8_t payload[] = "hello";
    stub_psm_set_data(payload, sizeof(payload) - 1);
    be_assert_success(
        "import path\n"
        "var ok = path.rename('/_matter_fabrics.tmp', "
        "'/_matter_fabrics.json')\n"
        "assert(ok == true, 'expected true, got: ' + str(ok))\n");
}

void test_path_rename_missing_source_returns_false(void) {
    stub_psm_clear();
    be_assert_success(
        "import path\n"
        "var ok = path.rename('/does_not_exist', '/dest')\n"
        "assert(ok == false, 'expected false, got: ' + str(ok))\n");
}

void test_path_rename_psm_error_returns_false(void) {
    const uint8_t payload[] = "hello";
    stub_psm_set_data(payload, sizeof(payload) - 1);
    stub_psm_set_error(-1);
    be_assert_success(
        "import path\n"
        "var ok = path.rename('/src', '/dst')\n"
        "assert(ok == false, 'expected false, got: ' + str(ok))\n");
    stub_psm_set_error(0);
}

void run_tests(void) {
    UnitySetTestFile(__FILE__);
    RUN_TEST(test_tasmota_config);
    RUN_TEST(test_tasmota_when_network_up_fires);
    RUN_TEST(test_tasmota_fast_loop_runs);
    RUN_TEST(test_persist);
    RUN_TEST(test_path_rename_success_returns_true);
    RUN_TEST(test_path_rename_missing_source_returns_false);
    RUN_TEST(test_path_rename_psm_error_returns_false);
}
