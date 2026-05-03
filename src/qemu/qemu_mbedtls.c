#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "FreeRTOS.h"
#include "app_crypto.h"
#include "entropy_poll.h"
#include "mbedtls/build_info.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/entropy.h"

#define SHA256_HASH_SIZE (32U)

typedef struct {
    mbedtls_entropy_context entropy;
    mbedtls_ctr_drbg_context ctrDrbg;
} mbedtls_gdata_t;

static mbedtls_gdata_t s_internalMbedtlsGdata;
static bool s_internalEntropyContextSet = false;
static uint8_t s_hashBuf[SHA256_HASH_SIZE];

static int internal_entropy_poll(void* data, unsigned char* output, size_t len,
                                 size_t* olen) {
    (void)data;
    size_t provided = 0;
    while (provided < len) {
        size_t to_copy = len - provided;
        if (to_copy > SHA256_HASH_SIZE) to_copy = SHA256_HASH_SIZE;
        memcpy(output + provided, s_hashBuf, to_copy);
        provided += to_copy;
    }
    *olen = provided;
    return 0;
}

void mbedtls_hardware_init_hash(uint8_t* entropy, size_t len) {
    size_t copy_len = len < SHA256_HASH_SIZE ? len : SHA256_HASH_SIZE;
    memcpy(s_hashBuf, entropy, copy_len);
    if (copy_len < SHA256_HASH_SIZE) {
        memset(s_hashBuf + copy_len, 0, SHA256_HASH_SIZE - copy_len);
    }
}

int mbedtls_hardware_poll(void* data, unsigned char* output, size_t len,
                          size_t* olen) {
    return internal_entropy_poll(data, output, len, olen);
}

static int internal_entropy_ctr_drbg_setup(void) {
    int ret = 0;

    if (!s_internalEntropyContextSet) {
        mbedtls_entropy_init(&s_internalMbedtlsGdata.entropy);

        /* In QEMU we don't have default hardware poll, but we'll add our own
         * source */
        mbedtls_entropy_add_source(
            &s_internalMbedtlsGdata.entropy, internal_entropy_poll, NULL,
            MBEDTLS_ENTROPY_MIN_HARDWARE, MBEDTLS_ENTROPY_SOURCE_STRONG);

        mbedtls_ctr_drbg_init(&s_internalMbedtlsGdata.ctrDrbg);

        if ((ret = mbedtls_ctr_drbg_seed(
                 &s_internalMbedtlsGdata.ctrDrbg, mbedtls_entropy_func,
                 &s_internalMbedtlsGdata.entropy, NULL, 0)) != 0) {
            printf("mbedtls_ctr_drbg_seed returned, ret = -0x%02X\n", -ret);
            return -1;
        }

        s_internalEntropyContextSet = true;
    }

    return 0;
}

mbedtls_ctr_drbg_context* app_get_global_drbg(void) {
    if (internal_entropy_ctr_drbg_setup() != 0) {
        return NULL;
    }
    return &s_internalMbedtlsGdata.ctrDrbg;
}

mbedtls_entropy_context* app_get_global_entropy(void) {
    if (internal_entropy_ctr_drbg_setup() != 0) {
        return NULL;
    }
    return &s_internalMbedtlsGdata.entropy;
}
