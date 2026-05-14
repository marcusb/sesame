/* Stubs for mbedTLS CCM streaming API.
 *
 * MBEDTLS_CCM_ALT defined in mbedtls_app_config.h disables all of mbedTLS's
 * ccm.c (the entire file is wrapped in #if !defined(MBEDTLS_CCM_ALT)).
 * The MW320 HW accelerator exposes only one-shot encrypt/decrypt; the streaming
 * update API is never called by CHIP or Sesame.  These stubs satisfy the
 * linker and assert-fail at runtime if somehow reached.
 */
#include <assert.h>
#include <mbedtls/build_info.h>
#include <stddef.h>

#ifdef MBEDTLS_CCM_ALT
#include <mbedtls/ccm.h>

int mbedtls_ccm_starts(mbedtls_ccm_context* ctx, int mode,
                       const unsigned char* iv, size_t iv_len) {
    (void)ctx;
    (void)mode;
    (void)iv;
    (void)iv_len;
    assert(0);
    return -1;
}

int mbedtls_ccm_set_lengths(mbedtls_ccm_context* ctx, size_t total_ad_len,
                            size_t plaintext_len, size_t tag_len) {
    (void)ctx;
    (void)total_ad_len;
    (void)plaintext_len;
    (void)tag_len;
    assert(0);
    return -1;
}

int mbedtls_ccm_update_ad(mbedtls_ccm_context* ctx, const unsigned char* ad,
                          size_t ad_len) {
    (void)ctx;
    (void)ad;
    (void)ad_len;
    assert(0);
    return -1;
}

int mbedtls_ccm_update(mbedtls_ccm_context* ctx, const unsigned char* input,
                       size_t input_len, unsigned char* output,
                       size_t output_size, size_t* output_len) {
    (void)ctx;
    (void)input;
    (void)input_len;
    (void)output;
    (void)output_size;
    (void)output_len;
    assert(0);
    return -1;
}

int mbedtls_ccm_finish(mbedtls_ccm_context* ctx, unsigned char* tag,
                       size_t tag_len) {
    (void)ctx;
    (void)tag;
    (void)tag_len;
    assert(0);
    return -1;
}
#endif /* MBEDTLS_CCM_ALT */
