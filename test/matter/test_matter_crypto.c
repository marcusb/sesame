#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "app_crypto.h"
#include "berry.h"
#include "matter_test_utils.h"
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

/* ECDSA sign + verify round-trip with a freshly-derived key. */
void test_crypto_ec_p256_ecdsa_roundtrip(void) {
    be_assert_success(
        "var ec = crypto.EC_P256() "
        "var priv = "
        "bytes('"
        "000102030405060708090a0b0c0d0e0f101112131415161718191a1b1c1d1e1f') "
        "var pub = ec.public_key(priv) "
        "assert(pub.size() == 65) "
        "var msg = bytes().fromstring('hello world') "
        "var sig = ec.ecdsa_sign_sha256(priv, msg) "
        "assert(sig.size() == 64) "
        "assert(ec.ecdsa_verify_sha256(pub, msg, sig)) "
        "var bad = bytes().fromstring('hello dolly') "
        "assert(!ec.ecdsa_verify_sha256(pub, bad, sig))");
}

/* ECDSA ASN.1 signing returns a valid DER SEQUENCE { r, s } (tag 0x30). */
void test_crypto_ec_p256_ecdsa_asn1(void) {
    be_assert_success(
        "var ec = crypto.EC_P256() "
        "var priv = "
        "bytes('"
        "0102030405060708010203040506070801020304050607080102030405060708') "
        "var msg = bytes().fromstring('matter') "
        "var sig = ec.ecdsa_sign_sha256_asn1(priv, msg) "
        "assert(sig[0] == 0x30) "
        "assert(size(sig) >= 8 && size(sig) <= 72)");
}

/* ECDH: a*B == b*A so shared_key is symmetric and equals the X coord of the
 * shared point. */
void test_crypto_ec_p256_shared_key(void) {
    be_assert_success(
        "var ec = crypto.EC_P256() "
        "var a_priv = "
        "bytes('"
        "1111111111111111111111111111111111111111111111111111111111111111') "
        "var b_priv = "
        "bytes('"
        "2222222222222222222222222222222222222222222222222222222222222222') "
        "var a_pub = ec.public_key(a_priv) "
        "var b_pub = ec.public_key(b_priv) "
        "var s_ab = ec.shared_key(a_priv, b_pub) "
        "var s_ba = ec.shared_key(b_priv, a_pub) "
        "assert(s_ab.size() == 32) "
        "assert(s_ab == s_ba)");
}

void run_tests(void) {
    UnitySetTestFile(__FILE__);
    RUN_TEST(test_crypto_sha256);
    RUN_TEST(test_crypto_sha256_oneshot_in_init);
    RUN_TEST(test_crypto_sha256_incremental_matches_oneshot);
    RUN_TEST(test_crypto_hmac);
    RUN_TEST(test_crypto_random);
    RUN_TEST(test_crypto_ec_p256_smoke);
    RUN_TEST(test_crypto_ec_p256_ecdsa_roundtrip);
    RUN_TEST(test_crypto_ec_p256_ecdsa_asn1);
    RUN_TEST(test_crypto_ec_p256_shared_key);
}
