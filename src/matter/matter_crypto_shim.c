/*
 * Matter Crypto Shims for Berry
 *
 * This module provides SHA256, HMAC-SHA256, Random, and NIST P-256 EC support.
 *
 * NOTE ON DYNAMIC LOADING:
 * The 'crypto' module and 'EC_P256' class are registered at runtime via
 * be_load_crypto_module() rather than using the Berry precompiled/static
 * table macros. This is a deliberate design choice to:
 *
 * 1. Simplify development: Avoids complex macro boilerplate for non-solidified
 * shims.
 * 2. Strict Stack Control: Manual registration allows us to explicitly call
 *    be_pop() after each be_setmember(). Previously, stack leakage during
 * static module construction caused the VM to misidentify the 'crypto' module,
 *    leading to hangs and memory corruption during heavy EC operations.
 * 3. Resource Management: Dynamic loading ensures that large mbedTLS structures
 *    like the EC group are only allocated and initialized when specifically
 *    requested by the VM.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_logging.h"
#define MATTER_LOG(fmt, ...) printf("[MATTER] " fmt "\r\n", ##__VA_ARGS__)
#include "be_mapping.h"
#include "berry.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/ecp.h"
#include "mbedtls/entropy.h"
#include "mbedtls/md.h"
#include "mbedtls/sha256.h"

static mbedtls_entropy_context entropy;
static mbedtls_ctr_drbg_context ctr_drbg;
static int drbg_initialized = 0;

static int ensure_drbg_init(void) {
    if (drbg_initialized) return 0;
    mbedtls_entropy_init(&entropy);
    mbedtls_ctr_drbg_init(&ctr_drbg);
    int ret = mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy,
                                    NULL, 0);
    if (ret == 0) drbg_initialized = 1;
    return ret;
}

/* SHA256(data:bytes) -> bytes(32) */
static int crypto_sha256(bvm* vm) {
    size_t len;
    const unsigned char* data = (const unsigned char*)be_tobytes(vm, 1, &len);
    unsigned char hash[32];
    mbedtls_sha256(data, len, hash, 0);
    be_pushbytes(vm, hash, 32);
    be_return(vm);
}

/* HMAC_SHA256(key:bytes, data:bytes) -> bytes(32) */
static int crypto_hmac_sha256(bvm* vm) {
    size_t key_len, data_len;
    const unsigned char* key =
        (const unsigned char*)be_tobytes(vm, 1, &key_len);
    const unsigned char* data =
        (const unsigned char*)be_tobytes(vm, 2, &data_len);
    unsigned char hmac[32];

    mbedtls_md_context_t ctx;
    mbedtls_md_init(&ctx);
    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
    mbedtls_md_hmac_starts(&ctx, key, key_len);
    mbedtls_md_hmac_update(&ctx, data, data_len);
    mbedtls_md_hmac_finish(&ctx, hmac);
    mbedtls_md_free(&ctx);

    be_pushbytes(vm, hmac, 32);
    be_return(vm);
}

/* random(size:int) -> bytes(size) */
static int crypto_random(bvm* vm) {
    int size = be_toint(vm, 1);
    if (size <= 0) be_return_nil(vm);
    if (ensure_drbg_init() != 0) be_return_nil(vm);

    unsigned char* buf = (unsigned char*)be_pushbytes(vm, NULL, size);
    if (mbedtls_ctr_drbg_random(&ctr_drbg, buf, size) != 0) {
        be_pop(vm, 1);
        be_return_nil(vm);
    }
    be_return(vm);
}

typedef struct {
    mbedtls_ecp_group grp;
} crypto_ec_p256_t;

/* EC_P256() constructor for the dynamic class */
static int crypto_ec_p256_init(bvm* vm) {
    crypto_ec_p256_t* ec = (crypto_ec_p256_t*)malloc(sizeof(crypto_ec_p256_t));
    mbedtls_ecp_group_init(&ec->grp);
    int ret = mbedtls_ecp_group_load(&ec->grp, MBEDTLS_ECP_DP_SECP256R1);
    if (ret != 0) {
        MATTER_LOG("mbedtls_ecp_group_load failed: %d", ret);
    }

    /* Store the instance in the class member '.p' */
    be_pushcomptr(vm, ec);
    be_setmember(vm, 1, ".p");

    be_return_nil(vm);
}

static int crypto_ec_p256_deinit(bvm* vm) {
    be_getmember(vm, 1, ".p");
    crypto_ec_p256_t* ec = (crypto_ec_p256_t*)be_tocomptr(vm, -1);
    if (ec) {
        mbedtls_ecp_group_free(&ec->grp);
        free(ec);
        be_pushcomptr(vm, NULL);
        be_setmember(vm, 1, ".p");
    }
    be_return_nil(vm);
}

/* mul(scalar:bytes) -> bytes(65) - multiply base point by scalar */
static int crypto_ec_p256_mul(bvm* vm) {
    be_getmember(vm, 1, ".p");
    crypto_ec_p256_t* ec = (crypto_ec_p256_t*)be_tocomptr(vm, -1);
    size_t scalar_len;
    const unsigned char* scalar_buf =
        (const unsigned char*)be_tobytes(vm, 2, &scalar_len);

    mbedtls_mpi d;
    mbedtls_ecp_point Q;
    mbedtls_mpi_init(&d);
    mbedtls_ecp_point_init(&Q);

    mbedtls_mpi_read_binary(&d, scalar_buf, scalar_len);

    if (ensure_drbg_init() != 0) {
        mbedtls_mpi_free(&d);
        mbedtls_ecp_point_free(&Q);
        be_return_nil(vm);
    }

    if (!drbg_initialized) {
        if (ensure_drbg_init() != 0) {
            MATTER_LOG("FAILED to initialize DRBG");
            mbedtls_mpi_free(&d);
            mbedtls_ecp_point_free(&Q);
            be_return_nil(vm);
        }
    }

    MATTER_LOG("Starting NIST P-256 scalar multiplication...");
    int ret = mbedtls_ecp_mul(&ec->grp, &Q, &d, &ec->grp.G,
                              mbedtls_ctr_drbg_random, &ctr_drbg);
    MATTER_LOG("mbedtls_ecp_mul finished: %d", ret);

    unsigned char out[65];
    size_t out_len = 0;
    if (ret == 0) {
        mbedtls_ecp_point_write_binary(&ec->grp, &Q,
                                       MBEDTLS_ECP_PF_UNCOMPRESSED, &out_len,
                                       out, sizeof(out));
    }

    mbedtls_mpi_free(&d);
    mbedtls_ecp_point_free(&Q);

    if (ret != 0) be_return_nil(vm);
    be_pushbytes(vm, out, out_len);
    be_return(vm);
}

/*
 * Runtime loader called by VM initialization
 */
int be_load_crypto_module(bvm* vm) {
    /* 1. Register EC_P256 class */
    static const bnfuncinfo ec_members[] = {
        {".p", NULL}, /* Storage for comptr */
        {"init", crypto_ec_p256_init},
        {"deinit", crypto_ec_p256_deinit},
        {"mul", crypto_ec_p256_mul},
        {NULL, NULL}};
    be_regclass(vm, "EC_P256", ec_members);

    /* 2. Create crypto module */
    be_newmodule(vm);
    be_setname(vm, -1, "crypto");

    be_pushntvfunction(vm, crypto_sha256);
    be_setmember(vm, -2, "SHA256");
    be_pop(vm, 1);

    be_pushntvfunction(vm, crypto_hmac_sha256);
    be_setmember(vm, -2, "HMAC_SHA256");
    be_pop(vm, 1);

    be_pushntvfunction(vm, crypto_random);
    be_setmember(vm, -2, "random");
    be_pop(vm, 1);

    /* Add the EC_P256 class to the module */
    be_getglobal(vm, "EC_P256");
    be_setmember(vm, -2, "EC_P256");
    be_pop(vm, 1);

    /* Register crypto in global scope */
    be_setglobal(vm, "crypto");

    return 0;
}
