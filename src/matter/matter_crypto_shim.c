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

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_logging.h"
#define MATTER_LOG(fmt, ...) LogInfo(("[MATTER] " fmt, ##__VA_ARGS__))
#include "be_class.h"
#include "be_mapping.h"
#include "be_module.h"
#include "be_object.h"
#include "be_string.h"
#include "be_vector.h"
#include "berry.h"
#include "mbedtls/ccm.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/error.h"

/* The mw320 mbedtls port defines MBEDTLS_CCM_ALT (see
 * include/config/mbedtls_app_config.h) which removes vanilla ccm.c streaming
 * functions, but mbedtls cipher.c still references mbedtls_ccm_update for the
 * MBEDTLS_MODE_CCM_STAR_NO_TAG cipher mode (a dead branch we never hit). Stub
 * it here so the linker is happy. */
#ifdef MBEDTLS_CCM_ALT
int mbedtls_ccm_update(mbedtls_ccm_context* ctx, const unsigned char* input,
                       size_t input_len, unsigned char* output,
                       size_t output_size, size_t* output_len) {
    (void)ctx;
    (void)input;
    (void)input_len;
    (void)output;
    (void)output_size;
    (void)output_len;
    assert(false);
}
#endif
#include "app_crypto.h"
#include "mbedtls/asn1.h"
#include "mbedtls/asn1write.h"
#include "mbedtls/ecdh.h"
#include "mbedtls/ecdsa.h"
#include "mbedtls/ecp.h"
#include "mbedtls/entropy.h"
#include "mbedtls/hkdf.h"
#include "mbedtls/md.h"
#include "mbedtls/pkcs5.h"
#include "mbedtls/sha256.h"

/* SHA256 / HMAC_SHA256 are exposed as classes in Tasmota — Berry code uses
 * `crypto.SHA256().update(d).update(d2).out()` (incremental). The one-shot
 * `crypto.SHA256(data)` form is also supported by detecting an arg in init. */

static int crypto_sha256_init(bvm* vm) {
    mbedtls_sha256_context* ctx = malloc(sizeof(*ctx));
    mbedtls_sha256_init(ctx);
    mbedtls_sha256_starts(ctx, 0);
    if (be_top(vm) >= 2 && be_isbytes(vm, 2)) {
        size_t len;
        const unsigned char* data =
            (const unsigned char*)be_tobytes(vm, 2, &len);
        mbedtls_sha256_update(ctx, data, len);
    }
    be_pushcomptr(vm, ctx);
    be_setmember(vm, 1, ".p");
    be_return_nil(vm);
}

static int crypto_sha256_deinit(bvm* vm) {
    be_getmember(vm, 1, ".p");
    mbedtls_sha256_context* ctx = (mbedtls_sha256_context*)be_tocomptr(vm, -1);
    if (ctx) {
        mbedtls_sha256_free(ctx);
        free(ctx);
        be_pushcomptr(vm, NULL);
        be_setmember(vm, 1, ".p");
    }
    be_return_nil(vm);
}

static int crypto_sha256_update(bvm* vm) {
    be_getmember(vm, 1, ".p");
    mbedtls_sha256_context* ctx = (mbedtls_sha256_context*)be_tocomptr(vm, -1);
    be_pop(vm, 1);
    size_t len;
    const unsigned char* data = (const unsigned char*)be_tobytes(vm, 2, &len);
    if (ctx && data) mbedtls_sha256_update(ctx, data, len);
    /* return self for chaining */
    be_pushvalue(vm, 1);
    be_return(vm);
}

static int crypto_sha256_out(bvm* vm) {
    be_getmember(vm, 1, ".p");
    mbedtls_sha256_context* ctx = (mbedtls_sha256_context*)be_tocomptr(vm, -1);
    be_pop(vm, 1);
    unsigned char out[32];
    if (!ctx) be_return_nil(vm);
    mbedtls_sha256_finish(ctx, out);
    /* Re-start so the instance is reusable as Tasmota's API expects. */
    mbedtls_sha256_starts(ctx, 0);
    be_pushbytes(vm, out, 32);
    be_return(vm);
}

typedef struct {
    mbedtls_md_context_t md;
} hmac_ctx_t;

static int crypto_hmac_init(bvm* vm) {
    hmac_ctx_t* ctx = malloc(sizeof(*ctx));
    mbedtls_md_init(&ctx->md);
    mbedtls_md_setup(&ctx->md, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
    if (be_top(vm) >= 2 && be_isbytes(vm, 2)) {
        size_t key_len;
        const unsigned char* key =
            (const unsigned char*)be_tobytes(vm, 2, &key_len);
        mbedtls_md_hmac_starts(&ctx->md, key, key_len);
    }
    be_pushcomptr(vm, ctx);
    be_setmember(vm, 1, ".p");
    be_return_nil(vm);
}

static int crypto_hmac_deinit(bvm* vm) {
    be_getmember(vm, 1, ".p");
    hmac_ctx_t* ctx = (hmac_ctx_t*)be_tocomptr(vm, -1);
    if (ctx) {
        mbedtls_md_free(&ctx->md);
        free(ctx);
        be_pushcomptr(vm, NULL);
        be_setmember(vm, 1, ".p");
    }
    be_return_nil(vm);
}

static int crypto_hmac_update(bvm* vm) {
    be_getmember(vm, 1, ".p");
    hmac_ctx_t* ctx = (hmac_ctx_t*)be_tocomptr(vm, -1);
    be_pop(vm, 1);
    size_t len;
    const unsigned char* data = (const unsigned char*)be_tobytes(vm, 2, &len);
    if (ctx && data) mbedtls_md_hmac_update(&ctx->md, data, len);
    be_pushvalue(vm, 1);
    be_return(vm);
}

static int crypto_hmac_out(bvm* vm) {
    be_getmember(vm, 1, ".p");
    hmac_ctx_t* ctx = (hmac_ctx_t*)be_tocomptr(vm, -1);
    be_pop(vm, 1);
    unsigned char out[32];
    if (!ctx) be_return_nil(vm);
    mbedtls_md_hmac_finish(&ctx->md, out);
    mbedtls_md_hmac_reset(&ctx->md);
    be_pushbytes(vm, out, 32);
    be_return(vm);
}

/* Tasmota exposes HKDF_SHA256 / PBKDF2_HMAC_SHA256 as classes whose only
 * method is `derive(...)`. Matter code calls
 * `crypto.HKDF_SHA256().derive(...)`, so shift arg indices to skip the implicit
 * `self`. */

/* HKDF_SHA256.derive(self, secret:bytes, salt:bytes, info:bytes, out_len:int)
 * -> bytes(out_len) */
static int crypto_hkdf_sha256(bvm* vm) {
    int base = (be_top(vm) >= 5) ? 1 : 0; /* skip self if instance-called */
    size_t secret_len, salt_len, info_len;
    const unsigned char* secret =
        (const unsigned char*)be_tobytes(vm, base + 1, &secret_len);
    const unsigned char* salt =
        (const unsigned char*)be_tobytes(vm, base + 2, &salt_len);
    const unsigned char* info =
        (const unsigned char*)be_tobytes(vm, base + 3, &info_len);
    int out_len = be_toint(vm, base + 4);

    if (!secret || out_len <= 0) be_return_nil(vm);

    unsigned char* output = (unsigned char*)be_pushbytes(vm, NULL, out_len);
    mbedtls_hkdf(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), salt, salt_len,
                 secret, secret_len, info, info_len, output, out_len);
    be_return(vm);
}

/* PBKDF2_HMAC_SHA256.derive(self, pass:bytes, salt:bytes, iter:int,
 * out_len:int) -> bytes(out_len) */
static int crypto_pbkdf2_hmac_sha256(bvm* vm) {
    int base = (be_top(vm) >= 5) ? 1 : 0;
    size_t pass_len, salt_len;
    const unsigned char* pass =
        (const unsigned char*)be_tobytes(vm, base + 1, &pass_len);
    const unsigned char* salt =
        (const unsigned char*)be_tobytes(vm, base + 2, &salt_len);
    int iter = be_toint(vm, base + 3);
    int out_len = be_toint(vm, base + 4);

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
    if (mbedtls_ctr_drbg_random(app_get_global_drbg(), buf, size) != 0) {
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

/* mod(x:bytes) -> bytes(32) — reduce x modulo the curve order n. Matter uses
 * this during SPAKE2+ setup (Matter_z_Commissioning.be). */
static int crypto_ec_p256_mod(bvm* vm) {
    be_getmember(vm, 1, ".p");
    crypto_ec_p256_t* ec = (crypto_ec_p256_t*)be_tocomptr(vm, -1);
    be_pop(vm, 1);

    size_t in_len;
    const unsigned char* in = (const unsigned char*)be_tobytes(vm, 2, &in_len);
    if (!ec || !in) be_return_nil(vm);

    mbedtls_mpi x;
    mbedtls_mpi_init(&x);
    int ret = mbedtls_mpi_read_binary(&x, in, in_len);
    if (ret == 0) ret = mbedtls_mpi_mod_mpi(&x, &x, &ec->grp.N);

    unsigned char out[32] = {0};
    if (ret == 0) ret = mbedtls_mpi_write_binary(&x, out, 32);
    mbedtls_mpi_free(&x);
    if (ret != 0) be_return_nil(vm);
    be_pushbytes(vm, out, 32);
    be_return(vm);
}

/* Read a point from bytes; treats an empty buffer as the generator G. */
static int load_point(crypto_ec_p256_t* ec, const unsigned char* buf,
                      size_t len, mbedtls_ecp_point* out) {
    if (len == 0) return mbedtls_ecp_copy(out, &ec->grp.G);
    return mbedtls_ecp_point_read_binary(&ec->grp, out, buf, len);
}

/* mul(scalar:bytes [, point:bytes]) -> bytes(65). With one arg, multiplies the
 * base point. With two args, multiplies the supplied point. */
static int crypto_ec_p256_mul(bvm* vm) {
    be_getmember(vm, 1, ".p");
    crypto_ec_p256_t* ec = (crypto_ec_p256_t*)be_tocomptr(vm, -1);
    be_pop(vm, 1);

    size_t scalar_len;
    const unsigned char* scalar_buf =
        (const unsigned char*)be_tobytes(vm, 2, &scalar_len);
    if (!ec || !scalar_buf) be_return_nil(vm);

    int top = be_top(vm);
    size_t pt_len = 0;
    const unsigned char* pt_buf = NULL;
    if (top >= 3 && be_isbytes(vm, 3)) {
        pt_buf = (const unsigned char*)be_tobytes(vm, 3, &pt_len);
    }

    mbedtls_mpi d;
    mbedtls_ecp_point P, Q;
    mbedtls_mpi_init(&d);
    mbedtls_ecp_point_init(&P);
    mbedtls_ecp_point_init(&Q);

    int ret = mbedtls_mpi_read_binary(&d, scalar_buf, scalar_len);
    if (ret == 0) {
        ret = pt_buf ? load_point(ec, pt_buf, pt_len, &P)
                     : mbedtls_ecp_copy(&P, &ec->grp.G);
    }
    if (ret == 0) {
        ret = mbedtls_ecp_mul(&ec->grp, &Q, &d, &P, mbedtls_ctr_drbg_random,
                              app_get_global_drbg());
    }

    unsigned char out[65];
    size_t out_len = 0;
    if (ret == 0) {
        ret = mbedtls_ecp_point_write_binary(&ec->grp, &Q,
                                             MBEDTLS_ECP_PF_UNCOMPRESSED,
                                             &out_len, out, sizeof(out));
    }

    mbedtls_mpi_free(&d);
    mbedtls_ecp_point_free(&P);
    mbedtls_ecp_point_free(&Q);

    if (ret != 0) be_return_nil(vm);
    be_pushbytes(vm, out, out_len);
    be_return(vm);
}

/* muladd(s1:bytes, P1:bytes, s2:bytes, P2:bytes) -> bytes(65). Computes
 * Q = s1*P1 + s2*P2. An empty P2 (or omitted) means the generator. SPAKE2+
 * uses this as `x*G + w0*M`. */
static int crypto_ec_p256_muladd(bvm* vm) {
    be_getmember(vm, 1, ".p");
    crypto_ec_p256_t* ec = (crypto_ec_p256_t*)be_tocomptr(vm, -1);
    be_pop(vm, 1);
    if (!ec) be_return_nil(vm);

    size_t s1_len, p1_len, s2_len, p2_len = 0;
    const unsigned char* s1_buf =
        (const unsigned char*)be_tobytes(vm, 2, &s1_len);
    const unsigned char* p1_buf =
        (const unsigned char*)be_tobytes(vm, 3, &p1_len);
    const unsigned char* s2_buf =
        (const unsigned char*)be_tobytes(vm, 4, &s2_len);
    const unsigned char* p2_buf = NULL;
    if (be_top(vm) >= 5 && be_isbytes(vm, 5)) {
        p2_buf = (const unsigned char*)be_tobytes(vm, 5, &p2_len);
    }
    if (!s1_buf || !p1_buf || !s2_buf) be_return_nil(vm);

    mbedtls_mpi m, n;
    mbedtls_ecp_point P1, P2, Q;
    mbedtls_mpi_init(&m);
    mbedtls_mpi_init(&n);
    mbedtls_ecp_point_init(&P1);
    mbedtls_ecp_point_init(&P2);
    mbedtls_ecp_point_init(&Q);

    int ret = mbedtls_mpi_read_binary(&m, s1_buf, s1_len);
    if (ret == 0) ret = mbedtls_mpi_read_binary(&n, s2_buf, s2_len);
    if (ret == 0) ret = load_point(ec, p1_buf, p1_len, &P1);
    if (ret == 0) {
        ret = p2_buf ? load_point(ec, p2_buf, p2_len, &P2)
                     : mbedtls_ecp_copy(&P2, &ec->grp.G);
    }
    if (ret == 0) ret = mbedtls_ecp_muladd(&ec->grp, &Q, &m, &P1, &n, &P2);

    unsigned char out[65];
    size_t out_len = 0;
    if (ret == 0) {
        ret = mbedtls_ecp_point_write_binary(&ec->grp, &Q,
                                             MBEDTLS_ECP_PF_UNCOMPRESSED,
                                             &out_len, out, sizeof(out));
    }

    mbedtls_mpi_free(&m);
    mbedtls_mpi_free(&n);
    mbedtls_ecp_point_free(&P1);
    mbedtls_ecp_point_free(&P2);
    mbedtls_ecp_point_free(&Q);

    if (ret != 0) be_return_nil(vm);
    be_pushbytes(vm, out, out_len);
    be_return(vm);
}

/* shared_key(priv:bytes(32), pub:bytes(65)) -> bytes(32). ECDH: returns the
 * X coordinate of priv * pub. */
static int crypto_ec_p256_shared_key(bvm* vm) {
    be_getmember(vm, 1, ".p");
    crypto_ec_p256_t* ec = (crypto_ec_p256_t*)be_tocomptr(vm, -1);
    be_pop(vm, 1);
    if (!ec) be_return_nil(vm);

    size_t priv_len, pub_len;
    const unsigned char* priv_buf =
        (const unsigned char*)be_tobytes(vm, 2, &priv_len);
    const unsigned char* pub_buf =
        (const unsigned char*)be_tobytes(vm, 3, &pub_len);
    if (!priv_buf || !pub_buf) be_return_nil(vm);

    mbedtls_mpi d, z;
    mbedtls_ecp_point Q;
    mbedtls_mpi_init(&d);
    mbedtls_mpi_init(&z);
    mbedtls_ecp_point_init(&Q);

    int ret = mbedtls_mpi_read_binary(&d, priv_buf, priv_len);
    if (ret == 0)
        ret = mbedtls_ecp_point_read_binary(&ec->grp, &Q, pub_buf, pub_len);
    if (ret == 0)
        ret = mbedtls_ecdh_compute_shared(&ec->grp, &z, &Q, &d,
                                          mbedtls_ctr_drbg_random,
                                          app_get_global_drbg());

    unsigned char out[32] = {0};
    if (ret == 0) ret = mbedtls_mpi_write_binary(&z, out, 32);

    mbedtls_mpi_free(&d);
    mbedtls_mpi_free(&z);
    mbedtls_ecp_point_free(&Q);

    if (ret != 0) be_return_nil(vm);
    be_pushbytes(vm, out, 32);
    be_return(vm);
}

/* Internal: compute SHA256(msg) and ECDSA-sign it with priv key, returning the
 * raw (r,s) MPIs for the caller to serialize. Returns 0 on success. */
static int ecdsa_sign_hashed(crypto_ec_p256_t* ec, const unsigned char* priv,
                             size_t priv_len, const unsigned char* msg,
                             size_t msg_len, mbedtls_mpi* r, mbedtls_mpi* s) {
    unsigned char hash[32];
    int ret = mbedtls_sha256(msg, msg_len, hash, 0);
    if (ret != 0) return ret;
    mbedtls_mpi d;
    mbedtls_mpi_init(&d);
    ret = mbedtls_mpi_read_binary(&d, priv, priv_len);
    if (ret == 0)
        ret =
            mbedtls_ecdsa_sign(&ec->grp, r, s, &d, hash, sizeof(hash),
                               mbedtls_ctr_drbg_random, app_get_global_drbg());
    mbedtls_mpi_free(&d);
    return ret;
}

/* ecdsa_sign_sha256(priv:bytes(32), msg:bytes()) -> bytes(64). Returns the
 * raw r||s P-256 ECDSA signature over SHA256(msg). */
static int crypto_ec_p256_ecdsa_sign_sha256(bvm* vm) {
    be_getmember(vm, 1, ".p");
    crypto_ec_p256_t* ec = (crypto_ec_p256_t*)be_tocomptr(vm, -1);
    be_pop(vm, 1);
    if (!ec) be_return_nil(vm);

    size_t priv_len, msg_len;
    const unsigned char* priv =
        (const unsigned char*)be_tobytes(vm, 2, &priv_len);
    const unsigned char* msg =
        (const unsigned char*)be_tobytes(vm, 3, &msg_len);
    if (!priv || !msg) be_return_nil(vm);

    mbedtls_mpi r, s;
    mbedtls_mpi_init(&r);
    mbedtls_mpi_init(&s);
    int ret = ecdsa_sign_hashed(ec, priv, priv_len, msg, msg_len, &r, &s);
    unsigned char out[64];
    if (ret == 0) ret = mbedtls_mpi_write_binary(&r, out, 32);
    if (ret == 0) ret = mbedtls_mpi_write_binary(&s, out + 32, 32);
    mbedtls_mpi_free(&r);
    mbedtls_mpi_free(&s);
    if (ret != 0) be_return_nil(vm);
    be_pushbytes(vm, out, 64);
    be_return(vm);
}

/* ecdsa_sign_sha256_asn1(priv:bytes(32), msg:bytes()) -> bytes. ASN.1 DER
 * encoded SEQUENCE { INTEGER r, INTEGER s }. */
static int crypto_ec_p256_ecdsa_sign_sha256_asn1(bvm* vm) {
    be_getmember(vm, 1, ".p");
    crypto_ec_p256_t* ec = (crypto_ec_p256_t*)be_tocomptr(vm, -1);
    be_pop(vm, 1);
    if (!ec) be_return_nil(vm);

    size_t priv_len, msg_len;
    const unsigned char* priv =
        (const unsigned char*)be_tobytes(vm, 2, &priv_len);
    const unsigned char* msg =
        (const unsigned char*)be_tobytes(vm, 3, &msg_len);
    if (!priv || !msg) be_return_nil(vm);

    mbedtls_mpi r, s;
    mbedtls_mpi_init(&r);
    mbedtls_mpi_init(&s);
    int ret = ecdsa_sign_hashed(ec, priv, priv_len, msg, msg_len, &r, &s);

    /* Encode r,s as ASN.1 DER. mbedtls_asn1_write_* functions write backwards
     * from the end of the buffer. Max P-256 sig: tag(1)+len(1)+9*2+32*2 ~ 72.
     */
    unsigned char buf[80];
    unsigned char* p = buf + sizeof(buf);
    int len = 0;
    if (ret == 0) {
        int n = mbedtls_asn1_write_mpi(&p, buf, &s);
        if (n < 0)
            ret = n;
        else
            len += n;
    }
    if (ret == 0) {
        int n = mbedtls_asn1_write_mpi(&p, buf, &r);
        if (n < 0)
            ret = n;
        else
            len += n;
    }
    if (ret == 0) {
        int n = mbedtls_asn1_write_len(&p, buf, len);
        if (n < 0)
            ret = n;
        else
            len += n;
    }
    if (ret == 0) {
        int n = mbedtls_asn1_write_tag(
            &p, buf, MBEDTLS_ASN1_CONSTRUCTED | MBEDTLS_ASN1_SEQUENCE);
        if (n < 0)
            ret = n;
        else
            len += n;
    }
    mbedtls_mpi_free(&r);
    mbedtls_mpi_free(&s);
    if (ret != 0) be_return_nil(vm);
    be_pushbytes(vm, p, len);
    be_return(vm);
}

/* ecdsa_verify_sha256(pub:bytes(65), msg:bytes(), sig:bytes(64)) -> bool.
 * Verifies a raw r||s signature over SHA256(msg) under the given public key. */
static int crypto_ec_p256_ecdsa_verify_sha256(bvm* vm) {
    be_getmember(vm, 1, ".p");
    crypto_ec_p256_t* ec = (crypto_ec_p256_t*)be_tocomptr(vm, -1);
    be_pop(vm, 1);
    if (!ec) {
        be_pushbool(vm, bfalse);
        be_return(vm);
    }

    size_t pub_len, msg_len, sig_len;
    const unsigned char* pub =
        (const unsigned char*)be_tobytes(vm, 2, &pub_len);
    const unsigned char* msg =
        (const unsigned char*)be_tobytes(vm, 3, &msg_len);
    const unsigned char* sig =
        (const unsigned char*)be_tobytes(vm, 4, &sig_len);
    if (!pub || !msg || !sig || sig_len != 64) {
        be_pushbool(vm, bfalse);
        be_return(vm);
    }

    unsigned char hash[32];
    int ret = mbedtls_sha256(msg, msg_len, hash, 0);

    mbedtls_ecp_point Q;
    mbedtls_mpi r, s;
    mbedtls_ecp_point_init(&Q);
    mbedtls_mpi_init(&r);
    mbedtls_mpi_init(&s);

    if (ret == 0)
        ret = mbedtls_ecp_point_read_binary(&ec->grp, &Q, pub, pub_len);
    if (ret == 0) ret = mbedtls_mpi_read_binary(&r, sig, 32);
    if (ret == 0) ret = mbedtls_mpi_read_binary(&s, sig + 32, 32);
    if (ret == 0)
        ret = mbedtls_ecdsa_verify(&ec->grp, hash, sizeof(hash), &Q, &r, &s);

    mbedtls_ecp_point_free(&Q);
    mbedtls_mpi_free(&r);
    mbedtls_mpi_free(&s);

    be_pushbool(vm, ret == 0 ? btrue : bfalse);
    be_return(vm);
}

/* AES_CCM with the Tasmota Berry surface that Matter uses:
 *   - constructor: AES_CCM(key, iv, aad, data_len, tag_len) -> instance
 *   - instance.encrypt(data) -> ciphertext (encrypted in one shot)
 *   - instance.decrypt(data) -> plaintext
 *   - instance.tag()         -> bytes(tag_len) computed by last encrypt/decrypt
 *   - static encrypt1(key, n_buf, n_off, n_len, aad_buf, aad_off, aad_len,
 *                     in_buf, in_off, in_len, tag_buf, tag_off, tag_len)
 *     decrypt1(...) — in-place secure-channel ops used by Matter_Message.be.
 * encrypt/decrypt store the resulting tag in the instance for tag() to read.
 */
typedef struct {
    mbedtls_ccm_context ccm;
    /* Init params we need to remember for the eventual encrypt/decrypt. */
    unsigned char iv[13];
    size_t iv_len;
    unsigned char aad[64];
    size_t aad_len;
    int tag_len;
    /* Tag computed by the most recent encrypt/decrypt. */
    unsigned char tag[16];
} aes_ccm_t;

static int crypto_aes_ccm_init(bvm* vm) {
    /* (1=self, 2=key, 3=iv, 4=aad, 5=data_len[unused], 6=tag_len) */
    size_t key_len, iv_len, aad_len;
    const unsigned char* key =
        (const unsigned char*)be_tobytes(vm, 2, &key_len);
    const unsigned char* iv = (const unsigned char*)be_tobytes(vm, 3, &iv_len);
    const unsigned char* aad =
        (const unsigned char*)be_tobytes(vm, 4, &aad_len);
    int tag_len = be_toint(vm, 6);
    aes_ccm_t* c = malloc(sizeof(*c));
    mbedtls_ccm_init(&c->ccm);
    mbedtls_ccm_setkey(&c->ccm, MBEDTLS_CIPHER_ID_AES, key, key_len * 8);
    if (iv_len > sizeof(c->iv)) iv_len = sizeof(c->iv);
    memcpy(c->iv, iv, iv_len);
    c->iv_len = iv_len;
    if (aad_len > sizeof(c->aad)) aad_len = sizeof(c->aad);
    memcpy(c->aad, aad ? aad : (const unsigned char*)"", aad_len);
    c->aad_len = aad_len;
    c->tag_len = tag_len;
    memset(c->tag, 0, sizeof(c->tag));
    be_pushcomptr(vm, c);
    be_setmember(vm, 1, ".p");
    be_return_nil(vm);
}

static int crypto_aes_ccm_deinit(bvm* vm) {
    be_getmember(vm, 1, ".p");
    aes_ccm_t* c = (aes_ccm_t*)be_tocomptr(vm, -1);
    if (c) {
        mbedtls_ccm_free(&c->ccm);
        free(c);
        be_pushcomptr(vm, NULL);
        be_setmember(vm, 1, ".p");
    }
    be_return_nil(vm);
}

static int aes_ccm_buf_aligned(const void* p) {
    return (((uintptr_t)p) & 3u) == 0u;
}

/* Run mbedtls_ccm_encrypt_and_tag / auth_decrypt with an existing context,
 * bouncing each input/output region through an aligned malloc'd buffer if the
 * caller-supplied pointer is byte-aligned. */
static int aes_ccm_run_ctx(mbedtls_ccm_context* ctx, int is_encrypt,
                           size_t in_len, const unsigned char* n, size_t n_len,
                           const unsigned char* aad, size_t aad_len,
                           const unsigned char* in, unsigned char* out,
                           unsigned char* tag, size_t tag_len) {
    int need_bounce = !aes_ccm_buf_aligned(n) || !aes_ccm_buf_aligned(aad) ||
                      !aes_ccm_buf_aligned(in) || !aes_ccm_buf_aligned(out) ||
                      !aes_ccm_buf_aligned(tag);

    if (!need_bounce) {
        return is_encrypt
                   ? mbedtls_ccm_encrypt_and_tag(ctx, in_len, n, n_len, aad,
                                                 aad_len, in, out, tag, tag_len)
                   : mbedtls_ccm_auth_decrypt(ctx, in_len, n, n_len, aad,
                                              aad_len, in, out, tag, tag_len);
    }

    /* Allocate aligned bounces (n is small + on-stack-aligned; AAD may be
     * empty). pvPortMalloc returns portBYTE_ALIGNMENT-aligned (>= 4) blocks. */
    unsigned char* bn = malloc(n_len ? n_len : 1);
    unsigned char* baad = aad_len ? malloc(aad_len) : NULL;
    unsigned char* bin = malloc(in_len ? in_len : 1);
    unsigned char* bout = malloc(in_len ? in_len : 1);
    unsigned char* btag = malloc(tag_len ? tag_len : 1);
    int ret = 0;

    if (!bn || (aad_len && !baad) || !bin || !bout || !btag) {
        ret = MBEDTLS_ERR_PLATFORM_HW_ACCEL_FAILED;
        goto cleanup;
    }
    memcpy(bn, n, n_len);
    if (aad_len) memcpy(baad, aad, aad_len);
    memcpy(bin, in, in_len);
    if (!is_encrypt) memcpy(btag, tag, tag_len);

    ret = is_encrypt
              ? mbedtls_ccm_encrypt_and_tag(ctx, in_len, bn, n_len, baad,
                                            aad_len, bin, bout, btag, tag_len)
              : mbedtls_ccm_auth_decrypt(ctx, in_len, bn, n_len, baad, aad_len,
                                         bin, bout, btag, tag_len);
    if (ret == 0) {
        memcpy(out, bout, in_len);
        if (is_encrypt) memcpy(tag, btag, tag_len);
    }

cleanup:
    free(bn);
    free(baad);
    free(bin);
    free(bout);
    free(btag);
    return ret;
}

static int crypto_aes_ccm_encrypt(bvm* vm) {
    be_getmember(vm, 1, ".p");
    aes_ccm_t* c = (aes_ccm_t*)be_tocomptr(vm, -1);
    be_pop(vm, 1);
    size_t in_len;
    const unsigned char* in = (const unsigned char*)be_tobytes(vm, 2, &in_len);
    if (!c || !in) be_return_nil(vm);
    unsigned char* out = (unsigned char*)be_pushbytes(vm, NULL, in_len);
    int ret = aes_ccm_run_ctx(&c->ccm, 1, in_len, c->iv, c->iv_len, c->aad,
                              c->aad_len, in, out, c->tag, c->tag_len);
    if (ret != 0) {
        be_pop(vm, 1);
        be_return_nil(vm);
    }
    be_return(vm);
}

static int crypto_aes_ccm_decrypt(bvm* vm) {
    be_getmember(vm, 1, ".p");
    aes_ccm_t* c = (aes_ccm_t*)be_tocomptr(vm, -1);
    be_pop(vm, 1);
    size_t in_len;
    const unsigned char* in = (const unsigned char*)be_tobytes(vm, 2, &in_len);
    if (!c || !in) be_return_nil(vm);
    /* The Berry caller splits the encrypted payload from the trailing tag and
     * compares the tag separately, so decrypt() must return plaintext even
     * when authentication fails. mbedtls_ccm_auth_decrypt zeroes the output
     * on tag mismatch, so we use a two-step path:
     *   1. encrypt_and_tag(ciphertext) → CTR is symmetric so this yields the
     *      plaintext in `out`. The tag here is meaningless (computed over the
     *      wrong material). Pop the throwaway tag.
     *   2. encrypt_and_tag(plaintext) → now we have the real expected tag for
     *      this plaintext, which tag() will return. */
    unsigned char* out = (unsigned char*)be_pushbytes(vm, NULL, in_len);
    unsigned char throwaway_tag[16];
    int ret = aes_ccm_run_ctx(&c->ccm, 1, in_len, c->iv, c->iv_len, c->aad,
                              c->aad_len, in, out, throwaway_tag, c->tag_len);
    if (ret != 0) {
        be_pop(vm, 1);
        be_return_nil(vm);
    }
    /* Recompute the tag from the recovered plaintext. We need a scratch
     * ciphertext buffer of the same size; allocate from the heap to avoid
     * stack pressure for larger payloads. */
    unsigned char* scratch = malloc(in_len);
    if (scratch) {
        aes_ccm_run_ctx(&c->ccm, 1, in_len, c->iv, c->iv_len, c->aad,
                        c->aad_len, out, scratch, c->tag, c->tag_len);
        free(scratch);
    }
    be_return(vm);
}

static int crypto_aes_ccm_tag(bvm* vm) {
    be_getmember(vm, 1, ".p");
    aes_ccm_t* c = (aes_ccm_t*)be_tocomptr(vm, -1);
    be_pop(vm, 1);
    if (!c) be_return_nil(vm);
    be_pushbytes(vm, c->tag, c->tag_len);
    be_return(vm);
}

/* Static encrypt1/decrypt1: in-place AES-CCM for secure-channel messages.
 * Args: (key, n_buf, n_off, n_len, aad_buf, aad_off, aad_len,
 *        in_buf, in_off, in_len, tag_buf, tag_off, tag_len)
 * Both calls modify in_buf in place; encrypt1 writes the resulting tag into
 * tag_buf at tag_off; decrypt1 returns true on auth success, false otherwise.
 */

/* Run mbedtls_ccm_encrypt_and_tag / auth_decrypt, bouncing each input/output
 * region through an aligned malloc'd buffer if the caller-supplied pointer
 * is byte-aligned. Returns 0 on success or a negative mbedtls error code (or
 * MBEDTLS_ERR_CCM_HW_ACCEL_FAILED on bounce-buffer OOM). */
static int aes_ccm_run(int is_encrypt, const unsigned char* key, size_t key_len,
                       const unsigned char* n, size_t n_len,
                       const unsigned char* aad, size_t aad_len,
                       unsigned char* io, size_t io_len, unsigned char* tag,
                       size_t tag_len) {
    mbedtls_ccm_context ctx;
    mbedtls_ccm_init(&ctx);
    int ret = mbedtls_ccm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, key, key_len * 8);
    if (ret != 0) {
        mbedtls_ccm_free(&ctx);
        return ret;
    }

    ret = aes_ccm_run_ctx(&ctx, is_encrypt, io_len, n, n_len, aad, aad_len, io,
                          io, tag, tag_len);

    mbedtls_ccm_free(&ctx);
    return ret;
}

static int crypto_aes_ccm_encrypt1(bvm* vm) {
    size_t key_len, n_len_b, aad_len_b, in_len_b, tag_len_b;
    const unsigned char* key =
        (const unsigned char*)be_tobytes(vm, 1, &key_len);
    const unsigned char* n_buf =
        (const unsigned char*)be_tobytes(vm, 2, &n_len_b);
    int n_off = be_toint(vm, 3), n_len = be_toint(vm, 4);
    unsigned char* aad_buf = (unsigned char*)be_tobytes(vm, 5, &aad_len_b);
    int aad_off = be_toint(vm, 6), aad_len = be_toint(vm, 7);
    unsigned char* in_buf = (unsigned char*)be_tobytes(vm, 8, &in_len_b);
    int in_off = be_toint(vm, 9), in_len = be_toint(vm, 10);
    unsigned char* tag_buf = (unsigned char*)be_tobytes(vm, 11, &tag_len_b);
    int tag_off = be_toint(vm, 12), tag_len = be_toint(vm, 13);
    if (!key || !n_buf || !in_buf || !tag_buf) be_return_nil(vm);

    int ret = aes_ccm_run(1, key, key_len, n_buf + n_off, n_len,
                          aad_buf + aad_off, aad_len, in_buf + in_off, in_len,
                          tag_buf + tag_off, tag_len);
    be_pushbool(vm, ret == 0 ? btrue : bfalse);
    be_return(vm);
}

static int crypto_aes_ccm_decrypt1(bvm* vm) {
    size_t key_len, n_len_b, aad_len_b, in_len_b, tag_len_b;
    const unsigned char* key =
        (const unsigned char*)be_tobytes(vm, 1, &key_len);
    const unsigned char* n_buf =
        (const unsigned char*)be_tobytes(vm, 2, &n_len_b);
    int n_off = be_toint(vm, 3), n_len = be_toint(vm, 4);
    unsigned char* aad_buf = (unsigned char*)be_tobytes(vm, 5, &aad_len_b);
    int aad_off = be_toint(vm, 6), aad_len = be_toint(vm, 7);
    unsigned char* in_buf = (unsigned char*)be_tobytes(vm, 8, &in_len_b);
    int in_off = be_toint(vm, 9), in_len = be_toint(vm, 10);
    unsigned char* tag_buf = (unsigned char*)be_tobytes(vm, 11, &tag_len_b);
    int tag_off = be_toint(vm, 12), tag_len = be_toint(vm, 13);
    if (!key || !n_buf || !in_buf || !tag_buf) be_return_nil(vm);

    int ret = aes_ccm_run(0, key, key_len, n_buf + n_off, n_len,
                          aad_buf + aad_off, aad_len, in_buf + in_off, in_len,
                          tag_buf + tag_off, tag_len);
    be_pushbool(vm, ret == 0 ? btrue : bfalse);
    be_return(vm);
}

/* neg(scalar:bytes) -> bytes(32) — returns (-scalar) mod n (curve order). */
static int crypto_ec_p256_neg(bvm* vm) {
    be_getmember(vm, 1, ".p");
    crypto_ec_p256_t* ec = (crypto_ec_p256_t*)be_tocomptr(vm, -1);
    be_pop(vm, 1);
    if (!ec) be_return_nil(vm);

    size_t in_len;
    const unsigned char* in = (const unsigned char*)be_tobytes(vm, 2, &in_len);
    if (!in) be_return_nil(vm);

    mbedtls_mpi x;
    mbedtls_mpi_init(&x);
    int ret = mbedtls_mpi_read_binary(&x, in, in_len);
    if (ret == 0) ret = mbedtls_mpi_mod_mpi(&x, &x, &ec->grp.N);
    if (ret == 0) ret = mbedtls_mpi_sub_mpi(&x, &ec->grp.N, &x);
    if (ret == 0) ret = mbedtls_mpi_mod_mpi(&x, &x, &ec->grp.N);

    unsigned char out[32] = {0};
    if (ret == 0) ret = mbedtls_mpi_write_binary(&x, out, 32);
    mbedtls_mpi_free(&x);
    if (ret != 0) be_return_nil(vm);
    be_pushbytes(vm, out, 32);
    be_return(vm);
}

/*
 * Runtime loader called by VM initialization
 */
int be_load_crypto_module(bvm* vm) {
    // register EC_P256 class
    static const bnfuncinfo ec_members[] = {
        {".p", NULL}, /* Storage for comptr */
        {"init", crypto_ec_p256_init},
        {"deinit", crypto_ec_p256_deinit},
        {"mul", crypto_ec_p256_mul},
        {"muladd", crypto_ec_p256_muladd},
        {"neg", crypto_ec_p256_neg},
        {"mod", crypto_ec_p256_mod},
        /* public_key(priv) = priv * G, which is exactly what mul() computes. */
        {"public_key", crypto_ec_p256_mul},
        {"shared_key", crypto_ec_p256_shared_key},
        {"ecdsa_sign_sha256", crypto_ec_p256_ecdsa_sign_sha256},
        {"ecdsa_sign_sha256_asn1", crypto_ec_p256_ecdsa_sign_sha256_asn1},
        {"ecdsa_verify_sha256", crypto_ec_p256_ecdsa_verify_sha256},
        {NULL, NULL}};
    be_regclass(vm, "EC_P256", ec_members);

    // create crypto module
    be_newmodule(vm);
    be_setname(vm, -1, "crypto");

    static const bnfuncinfo sha256_members[] = {
        {".p", NULL},
        {"init", crypto_sha256_init},
        {"deinit", crypto_sha256_deinit},
        {"update", crypto_sha256_update},
        {"out", crypto_sha256_out},
        {NULL, NULL}};
    be_regclass(vm, "SHA256", sha256_members);
    be_getglobal(vm, "SHA256");
    be_setmember(vm, -2, "SHA256");
    be_pop(vm, 1);

    static const bnfuncinfo hmac_members[] = {{".p", NULL},
                                              {"init", crypto_hmac_init},
                                              {"deinit", crypto_hmac_deinit},
                                              {"update", crypto_hmac_update},
                                              {"out", crypto_hmac_out},
                                              {NULL, NULL}};
    be_regclass(vm, "HMAC_SHA256", hmac_members);
    be_getglobal(vm, "HMAC_SHA256");
    be_setmember(vm, -2, "HMAC_SHA256");
    be_pop(vm, 1);

    /* HKDF_SHA256 / PBKDF2_HMAC_SHA256 must be classes so that code like
     * `crypto.HKDF_SHA256().derive(...)` works (Matter_Session.be, etc.). */
    static const bnfuncinfo hkdf_members[] = {{"derive", crypto_hkdf_sha256},
                                              {NULL, NULL}};
    be_regclass(vm, "HKDF_SHA256", hkdf_members);
    be_getglobal(vm, "HKDF_SHA256");
    be_setmember(vm, -2, "HKDF_SHA256");
    be_pop(vm, 1);

    static const bnfuncinfo pbkdf_members[] = {
        {"derive", crypto_pbkdf2_hmac_sha256}, {NULL, NULL}};
    be_regclass(vm, "PBKDF2_HMAC_SHA256", pbkdf_members);
    be_getglobal(vm, "PBKDF2_HMAC_SHA256");
    be_setmember(vm, -2, "PBKDF2_HMAC_SHA256");
    be_pop(vm, 1);

    static const bnfuncinfo aes_ccm_members[] = {
        {".p", NULL},
        {"init", crypto_aes_ccm_init},
        {"deinit", crypto_aes_ccm_deinit},
        {"encrypt", crypto_aes_ccm_encrypt},
        {"decrypt", crypto_aes_ccm_decrypt},
        {"tag", crypto_aes_ccm_tag},
        {NULL, NULL}};
    be_regclass(vm, "AES_CCM", aes_ccm_members);
    be_getglobal(vm, "AES_CCM");
    /* Attach encrypt1/decrypt1 to the AES_CCM class for Matter_Message.be's
     * `crypto.AES_CCM.encrypt1(...)` / `decrypt1(...)` call form. We use
     * be_class_native_method_bind here rather than be_setmember because the
     * latter requires the member to already exist (be_class_setmember only
     * updates, never inserts) and silently no-ops otherwise — leaving the
     * post-PASE encrypted-message decode failing with "no static attribute
     * 'decrypt1'" as soon as commissioning leaves the SPAKE2+ phase. */
    {
        bvalue* slot = be_indexof(vm, -1);
        bclass* aes_cls = var_toobj(slot);
        be_class_native_method_bind(vm, aes_cls, be_newstr(vm, "encrypt1"),
                                    crypto_aes_ccm_encrypt1);
        be_class_native_method_bind(vm, aes_cls, be_newstr(vm, "decrypt1"),
                                    crypto_aes_ccm_decrypt1);
    }
    be_setmember(vm, -2, "AES_CCM");
    be_pop(vm, 1);

    be_pushntvfunction(vm, crypto_random);
    be_setmember(vm, -2, "random");
    be_pop(vm, 1);

    /* Add the EC_P256 class to the module */
    be_getglobal(vm, "EC_P256");
    be_setmember(vm, -2, "EC_P256");
    be_pop(vm, 1);

    /* Register crypto in global scope AND in the module load cache so that
     * `import crypto` inside Berry code (e.g. solidified Matter_Device.init
     * at bytecode offset 0: IMPORT crypto) resolves to the same module. */
    be_cache_module(vm, be_newstr(vm, "crypto"));
    be_setglobal(vm, "crypto");

    return 0;
}
