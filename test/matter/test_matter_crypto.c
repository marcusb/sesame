#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "berry.h"
#include "matter_test_utils.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/ecp.h"
#include "mbedtls/entropy.h"
#include "task.h"
#include "unity.h"

void setUp(void) { matter_test_setup(); }
void tearDown(void) { matter_test_teardown(); }

void test_crypto_sha256(void) {
    be_assert_success(
        "var b = crypto.SHA256(bytes().fromstring('hello')); "
        "assert(b.tohex() == "
        "'2CF24DBA5FB0A30E26E83B2AC5B9E29E1B161E5C1FA7425E73043362938B9824')");
}

void test_crypto_hmac(void) {
    be_assert_success(
        "var key = bytes('0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b'); "
        "var data = bytes().fromstring('Hi There'); "
        "var h = crypto.HMAC_SHA256(key, data); "
        "assert(h.tohex() == "
        "'B0344C61D8DB38535CA8AFCEAF0BF12B881DC200C9833DA726E9376C2E32CFF7')");
}

void test_crypto_random(void) {
    be_assert_success("var b = crypto.random(16); assert(b.size() == 16)");
}

void test_crypto_ec_p256_smoke(void) {
    printf("Starting EC P256 smoke test from Berry...\n");
    be_assert_success(
        "var ec = crypto.EC_P256(); "
        "var scalar = "
        "bytes('"
        "000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f'); "
        "var Q = ec.mul(scalar); "
        "print('EC Mul result size:', Q.size()); "
        "assert(Q.size() == 65)");
    printf("EC P256 smoke test passed.\n");
}

extern void test_board_init(void);

static void run_tests_task(void* params) {
    (void)params;
    vTaskDelay(pdMS_TO_TICKS(2000));

    UNITY_BEGIN();
    RUN_TEST(test_crypto_sha256);
    RUN_TEST(test_crypto_hmac);
    RUN_TEST(test_crypto_random);
    RUN_TEST(test_crypto_ec_p256_smoke);
    int result = UNITY_END();
    printf("\r\nTEST_RESULT:%d\r\n", result);
    vTaskDelete(NULL);
}

int main(void) {
    test_board_init();
    if (xTaskCreate(run_tests_task, "tests", 4096, NULL, tskIDLE_PRIORITY + 1,
                    NULL) != pdPASS) {
        return 1;
    }
    vTaskStartScheduler();
    return 0;
}
