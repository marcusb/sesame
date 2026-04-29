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

/* AES-CCM encrypt1/decrypt1 round-trip with everything starting at offset 0
 * — exercises the fast path where aes_ccm_run() hands the caller's buffers
 * straight to mbedtls. */
void test_crypto_aes_ccm_static_aligned_roundtrip(void) {
    be_assert_success(
        "var key = bytes('00112233445566778899aabbccddeeff') "
        "var n = bytes('0102030405060708090a0b0c0d') "
        "var aad = bytes('aaaaaaaa') "
        "var pt = bytes('48656c6c6f2057' .. '6f726c6420212121') " /* 'Hello
                                                                     World !!!'
                                                                   */
        "var ct = pt.copy() "
        "var tag = bytes(-16) "
        "assert(crypto.AES_CCM.encrypt1(key, n, 0, size(n), aad, 0, size(aad),"
        " ct, 0, size(ct), tag, 0, 16)) "
        "assert(ct != pt) "
        "var pt2 = ct.copy() "
        "assert(crypto.AES_CCM.decrypt1(key, n, 0, size(n), aad, 0, size(aad),"
        " pt2, 0, size(pt2), tag, 0, 16)) "
        "assert(pt2 == pt)");
}

/* Same round-trip but with byte-aligned (offset != 0) AAD/payload/tag — this
 * is what Matter actually does: the encrypted region sits inside the wire
 * buffer at offset payload_idx (typically 8/16/18 bytes).  Forces the bounce
 * path inside aes_ccm_run() and verifies that HW CCM (when enabled) doesn't
 * fault on Cortex-M4 unaligned word loads.
 *
 * Strategy: encrypt the same plaintext twice — once with aligned offsets
 * (offset 0) and once with unaligned offsets (offset 1) — and assert that
 * the resulting ciphertext+tag are byte-identical, so the bounce path
 * matches the fast path. */
void test_crypto_aes_ccm_static_unaligned_roundtrip(void) {
    be_assert_success(
        "var key = bytes('00112233445566778899aabbccddeeff') "
        "var n   = bytes('0102030405060708090a0b0c0d') "
        "var aad = bytes('aaaaaaaa') "
        "var pt  = bytes('48656c6c6f20576f726c64212121') " /* 14 B */

        /* Aligned reference: 14 B input + 16 B tag, all at offset 0. */
        "var ref_ct  = pt.copy() "
        "var ref_tag = bytes(-16) "
        "assert(crypto.AES_CCM.encrypt1(key, n, 0, size(n), aad, 0, size(aad),"
        " ref_ct, 0, size(ref_ct), ref_tag, 0, 16),"
        " 'aligned encrypt1 failed') "

        /* Same params, but stored 1 byte into longer host buffers so every
         * offset is byte-aligned. */
        "var n_un = bytes(-1) + n " /* prefix 1 zero byte */
        "var aad_un = bytes(-1) + aad "
        "var io = bytes(-1) + pt + bytes(-16) " /* pt at off 1, tag at off 15 */
        "assert(size(n_un) == size(n) + 1, 'n_un size wrong: '..size(n_un)) "
        "assert(size(aad_un) == size(aad) + 1, 'aad_un size wrong: "
        "'..size(aad_un)) "
        "assert(size(io) == size(pt) + 17, 'io size wrong: '..size(io)) "
        "assert(crypto.AES_CCM.encrypt1(key, n_un, 1, size(n), aad_un, 1, "
        "size(aad), io, 1, size(pt), io, 1 + size(pt), 16),"
        " 'unaligned encrypt1 failed') "
        "var got_ct  = io[1 .. size(pt)] "
        "var got_tag = io[1 + size(pt) .. size(io) - 1] "
        "assert(got_ct == ref_ct, 'ct mismatch: '..got_ct.tohex()..' vs "
        "'..ref_ct.tohex()) "
        "assert(got_tag == ref_tag, 'tag mismatch: '..got_tag.tohex()..' vs "
        "'..ref_tag.tohex()) "

        /* And decrypt the unaligned buffer back to plaintext. */
        "assert(crypto.AES_CCM.decrypt1(key, n_un, 1, size(n), aad_un, 1, "
        "size(aad), io, 1, size(pt), io, 1 + size(pt), 16),"
        " 'unaligned decrypt1 failed') "
        "assert(io[1 .. size(pt)] == pt, 'pt mismatch')");
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
    RUN_TEST(test_crypto_aes_ccm_static_aligned_roundtrip);
    RUN_TEST(test_crypto_aes_ccm_static_unaligned_roundtrip);
}
