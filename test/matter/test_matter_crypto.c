#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
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
        "var b = crypto.SHA256().update(bytes().fromstring('hello')).out(); "
        "assert(b.tohex() == "
        "'2CF24DBA5FB0A30E26E83B2AC5B9E29E1B161E5C1FA7425E73043362938B9824')");
}

void test_crypto_sha256_oneshot_in_init(void) {
    be_assert_success(
        "var b = crypto.SHA256(bytes().fromstring('hello')).out(); "
        "assert(b.tohex() == "
        "'2CF24DBA5FB0A30E26E83B2AC5B9E29E1B161E5C1FA7425E73043362938B9824')");
}

void test_crypto_sha256_incremental_matches_oneshot(void) {
    be_assert_success(
        "var h = crypto.SHA256() "
        "h.update(bytes().fromstring('hel')) "
        "h.update(bytes().fromstring('lo')) "
        "var a = h.out() "
        "var b = crypto.SHA256(bytes().fromstring('hello')).out() "
        "assert(a == b)");
}

void test_crypto_hmac(void) {
    be_assert_success(
        "var key = bytes('0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b0b'); "
        "var data = bytes().fromstring('Hi There'); "
        "var h = crypto.HMAC_SHA256(key).update(data).out(); "
        "assert(h.tohex() == "
        "'B0344C61D8DB38535CA8AFCEAF0BF12B881DC200C9833DA726E9376C2E32CFF7')");
}

void test_crypto_random(void) {
    be_assert_success("var b = crypto.random(16); assert(b.size() == 16)");
}

void test_crypto_ec_p256_smoke(void) {
    be_assert_success(
        "var ec = crypto.EC_P256(); "
        "var scalar = "
        "bytes('"
        "000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f'); "
        "var Q = ec.mul(scalar); "
        "assert(Q.size() == 65)");
}

void run_tests(void) {
    UnitySetTestFile(__FILE__);
    RUN_TEST(test_crypto_sha256);
    RUN_TEST(test_crypto_sha256_oneshot_in_init);
    RUN_TEST(test_crypto_sha256_incremental_matches_oneshot);
    RUN_TEST(test_crypto_hmac);
    RUN_TEST(test_crypto_random);
    RUN_TEST(test_crypto_ec_p256_smoke);
}
