/*
 * Matter Crypto Shims for Berry
 *
 * This module provides SHA256, HMAC-SHA256, HKDF, PBKDF2, Random, and NIST
 * P-256 EC support.
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
#include "mbedtls/ecp.h"
#include "mbedtls/hkdf.h"
#include "mbedtls/md.h"
#include "mbedtls/pkcs5.h"
#include "mbedtls/sha256.h"

extern int get_drbg_random(void* p_rng, unsigned char* output, size_t len);

/* SHA256(data:bytes) -> bytes(32) */
static int crypto_sha256(bvm* vm) {
    size_t len;
    const unsigned char* data = (const unsigned char*)be_tobytes(vm, 1, &len);
    if (!data) be_return_nil(vm);

    unsigned char* output = (unsigned char*)be_pushbytes(vm, NULL, 32);
    mbedtls_sha256(data, len, output, 0);
    be_return(vm);
}

/* HMAC_SHA256(key:bytes, msg:bytes) -> bytes(32) */
static int crypto_hmac_sha256(bvm* vm) {
    size_t key_len, msg_len;
    const unsigned char* key =
        (const unsigned char*)be_tobytes(vm, 1, &key_len);
    const unsigned char* msg =
        (const unsigned char*)be_tobytes(vm, 2, &msg_len);
    if (!key || !msg) be_return_nil(vm);

    unsigned char* output = (unsigned char*)be_pushbytes(vm, NULL, 32);
    mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), key, key_len,
                    msg, msg_len, output);
    be_return(vm);
}

/* HKDF_SHA256(secret:bytes, salt:bytes, info:bytes, out_len:int) ->
 * bytes(out_len) */
static int crypto_hkdf_sha256(bvm* vm) {
    size_t secret_len, salt_len, info_len;
    const unsigned char* secret =
        (const unsigned char*)be_tobytes(vm, 1, &secret_len);
    const unsigned char* salt =
        (const unsigned char*)be_tobytes(vm, 2, &salt_len);
    const unsigned char* info =
        (const unsigned char*)be_tobytes(vm, 3, &info_len);
    int out_len = be_toint(vm, 4);

    if (!secret || out_len <= 0) be_return_nil(vm);

    unsigned char* output = (unsigned char*)be_pushbytes(vm, NULL, out_len);
    mbedtls_hkdf(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), salt, salt_len,
                 secret, secret_len, info, info_len, output, out_len);
    be_return(vm);
}

/* PBKDF2_HMAC_SHA256(pass:bytes, salt:bytes, iter:int, out_len:int) ->
 * bytes(out_len) */
static int crypto_pbkdf2_hmac_sha256(bvm* vm) {
    size_t pass_len, salt_len;
    const unsigned char* pass =
        (const unsigned char*)be_tobytes(vm, 1, &pass_len);
    const unsigned char* salt =
        (const unsigned char*)be_tobytes(vm, 2, &salt_len);
    int iter = be_toint(vm, 3);
    int out_len = be_toint(vm, 4);

    if (!pass || !salt || out_len <= 0) be_return_nil(vm);

    unsigned char* output = (unsigned char*)be_pushbytes(vm, NULL, out_len);
    mbedtls_pkcs5_pbkdf2_hmac_ext(MBEDTLS_MD_SHA256, pass, pass_len, salt,
                                  salt_len, iter, out_len, output);
    be_return(vm);
}

/* random(size:int) -> bytes(size) */
static int crypto_random(bvm* vm) {
    int size = be_toint(vm, 1);
    if (size <= 0) be_return_nil(vm);

    unsigned char* buf = (unsigned char*)be_pushbytes(vm, NULL, size);
    if (get_drbg_random(NULL, buf, size) != 0) {
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
    be_pop(vm, 1);

    size_t scalar_len;
    const unsigned char* scalar_buf =
        (const unsigned char*)be_tobytes(vm, 2, &scalar_len);
    if (!ec || !scalar_buf) be_return_nil(vm);

    mbedtls_mpi d;
    mbedtls_ecp_point Q;
    mbedtls_mpi_init(&d);
    mbedtls_ecp_point_init(&Q);

    if (mbedtls_mpi_read_binary(&d, scalar_buf, scalar_len) != 0) {
        mbedtls_mpi_free(&d);
        mbedtls_ecp_point_free(&Q);
        be_return_nil(vm);
    }

    MATTER_LOG("Starting NIST P-256 scalar multiplication...");
    int ret =
        mbedtls_ecp_mul(&ec->grp, &Q, &d, &ec->grp.G, get_drbg_random, NULL);
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

    be_pushntvfunction(vm, crypto_hkdf_sha256);
    be_setmember(vm, -2, "HKDF_SHA256");
    be_pop(vm, 1);

    be_pushntvfunction(vm, crypto_pbkdf2_hmac_sha256);
    be_setmember(vm, -2, "PBKDF2_HMAC_SHA256");
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