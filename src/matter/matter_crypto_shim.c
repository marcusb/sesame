#include <string.h>

#include "app_logging.h"
#undef LOG_INFO
#undef LOG_ERROR
#undef LOG_WARN
#undef LOG_DEBUG
#define LOG_INFO(fmt, ...) LogInfo((fmt, ##__VA_ARGS__))
#define LOG_ERROR(fmt, ...) LogError((fmt, ##__VA_ARGS__))
#define LOG_WARN(fmt, ...) LogWarn((fmt, ##__VA_ARGS__))
#define LOG_DEBUG(fmt, ...) LogDebug((fmt, ##__VA_ARGS__))
#include "be_constobj.h"
#include "be_mapping.h"
#include "be_mem.h"
#include "mbedtls/aes.h"
#include "mbedtls/ccm.h"
#include "mbedtls/ctr_drbg.h"
#include "mbedtls/ecp.h"
#include "mbedtls/entropy.h"
#include "mbedtls/hkdf.h"
#include "mbedtls/md.h"
#include "mbedtls/pkcs5.h"
#include "mbedtls/sha256.h"
#include "mbedtls/version.h"

#define TAG "matter_crypto"

// Static entropy and DRBG contexts for random number generation
static mbedtls_entropy_context entropy;
static mbedtls_ctr_drbg_context ctr_drbg;
static int drbg_init_done = 0;

static int ensure_drbg_init(void) {
    if (!drbg_init_done) {
        mbedtls_entropy_init(&entropy);
        mbedtls_ctr_drbg_init(&ctr_drbg);
        const char* pers = "matter_berry";
        int ret =
            mbedtls_ctr_drbg_seed(&ctr_drbg, mbedtls_entropy_func, &entropy,
                                  (const unsigned char*)pers, strlen(pers));
        if (ret != 0) {
            LOG_ERROR("mbedtls_ctr_drbg_seed failed: -0x%04x", -ret);
            return ret;
        }
        drbg_init_done = 1;
    }
    return 0;
}

/* crypto.random(len:int) -> bytes */
static int crypto_random(bvm* vm) {
    int32_t len = be_toint(vm, 1);
    if (ensure_drbg_init() != 0) {
        be_return_nil(vm);
    }

    void* buf = be_pushbuffer(vm, len);
    int ret = mbedtls_ctr_drbg_random(&ctr_drbg, (unsigned char*)buf, len);
    if (ret != 0) {
        LOG_ERROR("mbedtls_ctr_drbg_random failed: -0x%04x", -ret);
        be_return_nil(vm);
    }
    be_return(vm);
}

/* crypto.SHA256(data:bytes) -> bytes(32) */
static int crypto_sha256(bvm* vm) {
    size_t len;
    const void* data = be_tobytes(vm, 1, &len);
    unsigned char hash[32];

#if MBEDTLS_VERSION_MAJOR >= 3
    int ret = mbedtls_sha256(data, len, hash, 0);
#else
    int ret = mbedtls_sha256_ret(data, len, hash, 0);
#endif
    if (ret != 0) {
        LOG_ERROR("mbedtls_sha256 failed: -0x%04x", -ret);
        be_return_nil(vm);
    }
    be_pushbytes(vm, hash, 32);
    be_return(vm);
}

/* crypto.HMAC_SHA256(key:bytes, data:bytes) -> bytes(32) */
static int crypto_hmac_sha256(bvm* vm) {
    size_t key_len, data_len;
    const void* key = be_tobytes(vm, 1, &key_len);
    const void* data = be_tobytes(vm, 2, &data_len);
    unsigned char hash[32];

    const mbedtls_md_info_t* md_info =
        mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    int ret = mbedtls_md_hmac(md_info, key, key_len, data, data_len, hash);
    if (ret != 0) {
        LOG_ERROR("mbedtls_md_hmac failed: -0x%04x", -ret);
        be_return_nil(vm);
    }
    be_pushbytes(vm, hash, 32);
    be_return(vm);
}

/* crypto.HKDF_SHA256(salt:bytes, ikm:bytes, info:bytes, len:int) -> bytes */
static int crypto_hkdf_sha256(bvm* vm) {
    size_t salt_len, ikm_len, info_len;
    const void* salt = be_tobytes(vm, 1, &salt_len);
    const void* ikm = be_tobytes(vm, 2, &ikm_len);
    const void* info = be_tobytes(vm, 3, &info_len);
    int32_t out_len = be_toint(vm, 4);

    void* out = be_pushbuffer(vm, out_len);
    const mbedtls_md_info_t* md_info =
        mbedtls_md_info_from_type(MBEDTLS_MD_SHA256);
    int ret = mbedtls_hkdf(md_info, salt, salt_len, ikm, ikm_len, info,
                           info_len, out, out_len);
    if (ret != 0) {
        LOG_ERROR("mbedtls_hkdf failed: -0x%04x", -ret);
        be_return_nil(vm);
    }
    be_return(vm);
}

/* crypto.PBKDF2_HMAC_SHA256(pw:bytes, salt:bytes, iter:int, len:int) -> bytes
 */
static int crypto_pbkdf2_hmac_sha256(bvm* vm) {
    size_t pw_len, salt_len;
    const void* pw = be_tobytes(vm, 1, &pw_len);
    const void* salt = be_tobytes(vm, 2, &salt_len);
    int32_t iter = be_toint(vm, 3);
    int32_t out_len = be_toint(vm, 4);

    void* out = be_pushbuffer(vm, out_len);
    int ret;
#if MBEDTLS_VERSION_MAJOR >= 3
    ret = mbedtls_pkcs5_pbkdf2_hmac_ext(MBEDTLS_MD_SHA256, pw, pw_len, salt,
                                        salt_len, iter, out_len, out);
#else
    mbedtls_md_context_t sha_ctx;
    mbedtls_md_init(&sha_ctx);
    ret = mbedtls_md_setup(&sha_ctx,
                           mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
    if (ret == 0) {
        ret = mbedtls_pkcs5_pbkdf2_hmac(&sha_ctx, pw, pw_len, salt, salt_len,
                                        iter, out_len, out);
    }
    mbedtls_md_free(&sha_ctx);
#endif
    if (ret != 0) {
        LOG_ERROR("mbedtls_pkcs5_pbkdf2_hmac failed: -0x%04x", -ret);
        be_return_nil(vm);
    }
    be_return(vm);
}

/* crypto.AES_CCM encrypt/decrypt - simplified for Matter */
static int crypto_aes_ccm_crypt(bvm* vm, bbool encrypt) {
    size_t key_len, nonce_len, ad_len, data_len;
    const void* key = be_tobytes(vm, 1, &key_len);
    const void* nonce = be_tobytes(vm, 2, &nonce_len);
    const void* ad = be_tobytes(vm, 3, &ad_len);
    const void* data = be_tobytes(vm, 4, &data_len);
    int32_t tag_len = be_toint(vm, 5);

    mbedtls_ccm_context ctx;
    mbedtls_ccm_init(&ctx);
    mbedtls_ccm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, key, key_len * 8);

    int ret;
    if (encrypt) {
        void* out = be_pushbuffer(vm, data_len + tag_len);
        ret = mbedtls_ccm_encrypt_and_tag(
            &ctx, data_len, nonce, nonce_len, ad, ad_len, data, out,
            (unsigned char*)out + data_len, tag_len);
    } else {
        if (data_len < (size_t)tag_len) {
            mbedtls_ccm_free(&ctx);
            be_return_nil(vm);
        }
        size_t clear_len = data_len - tag_len;
        void* out = be_pushbuffer(vm, clear_len);
        ret = mbedtls_ccm_auth_decrypt(
            &ctx, clear_len, nonce, nonce_len, ad, ad_len, data, out,
            (const unsigned char*)data + clear_len, tag_len);
    }
    mbedtls_ccm_free(&ctx);
    if (ret != 0) {
        LOG_ERROR("AES-CCM %s failed: -0x%04x", encrypt ? "encrypt" : "decrypt",
                  -ret);
        be_return_nil(vm);
    }
    be_return(vm);
}

static int crypto_aes_ccm_encrypt(bvm* vm) {
    return crypto_aes_ccm_crypt(vm, btrue);
}

static int crypto_aes_ccm_decrypt(bvm* vm) {
    return crypto_aes_ccm_crypt(vm, bfalse);
}

/* crypto.EC_P256 - simplified for SPAKE2P */

typedef struct {
    mbedtls_ecp_group grp;
} crypto_ec_p256_t;

static int crypto_ec_p256_init(bvm* vm) {
    crypto_ec_p256_t* ec =
        (crypto_ec_p256_t*)be_malloc(vm, sizeof(crypto_ec_p256_t));
    mbedtls_ecp_group_init(&ec->grp);
    mbedtls_ecp_group_load(&ec->grp, MBEDTLS_ECP_DP_SECP256R1);
    be_pushcomptr(vm, ec);
    be_return(vm);
}

static int crypto_ec_p256_deinit(bvm* vm) {
    crypto_ec_p256_t* ec = (crypto_ec_p256_t*)be_tocomptr(vm, 1);
    mbedtls_ecp_group_free(&ec->grp);
    be_free(vm, ec, sizeof(crypto_ec_p256_t));
    be_return_nil(vm);
}

/* mul(scalar:bytes) -> bytes(65) - multiply base point by scalar */
static int crypto_ec_p256_mul(bvm* vm) {
    crypto_ec_p256_t* ec = (crypto_ec_p256_t*)be_tocomptr(vm, 1);
    size_t scalar_len;
    const unsigned char* scalar_buf =
        (const unsigned char*)be_tobytes(vm, 2, &scalar_len);

    mbedtls_mpi d;
    mbedtls_ecp_point Q;
    mbedtls_mpi_init(&d);
    mbedtls_ecp_point_init(&Q);

    mbedtls_mpi_read_binary(&d, scalar_buf, scalar_len);

    if (ensure_drbg_init() != 0) {
        be_return_nil(vm);
    }
    mbedtls_ecp_mul(&ec->grp, &Q, &d, &ec->grp.G, mbedtls_ctr_drbg_random,
                    &ctr_drbg);

    unsigned char out[65];
    size_t out_len;
    mbedtls_ecp_point_write_binary(&ec->grp, &Q, MBEDTLS_ECP_PF_UNCOMPRESSED,
                                   &out_len, out, sizeof(out));

    mbedtls_mpi_free(&d);
    mbedtls_ecp_point_free(&Q);

    void* res = be_pushbuffer(vm, out_len);
    memcpy(res, out, out_len);
    be_return(vm);
}

/* mul_point(point:bytes, scalar:bytes) -> bytes(65) - multiply arbitrary point
 * by scalar */
static int crypto_ec_p256_mul_point(bvm* vm) {
    crypto_ec_p256_t* ec = (crypto_ec_p256_t*)be_tocomptr(vm, 1);
    size_t point_len, scalar_len;
    const unsigned char* point_buf =
        (const unsigned char*)be_tobytes(vm, 2, &point_len);
    const unsigned char* scalar_buf =
        (const unsigned char*)be_tobytes(vm, 3, &scalar_len);

    mbedtls_mpi d;
    mbedtls_ecp_point P, Q;
    mbedtls_mpi_init(&d);
    mbedtls_ecp_point_init(&P);
    mbedtls_ecp_point_init(&Q);

    mbedtls_ecp_point_read_binary(&ec->grp, &P, point_buf, point_len);
    mbedtls_mpi_read_binary(&d, scalar_buf, scalar_len);

    if (ensure_drbg_init() != 0) {
        be_return_nil(vm);
    }
    mbedtls_ecp_mul(&ec->grp, &Q, &d, &P, mbedtls_ctr_drbg_random, &ctr_drbg);

    unsigned char out[65];
    size_t out_len;
    mbedtls_ecp_point_write_binary(&ec->grp, &Q, MBEDTLS_ECP_PF_UNCOMPRESSED,
                                   &out_len, out, sizeof(out));

    mbedtls_mpi_free(&d);
    mbedtls_ecp_point_free(&P);
    mbedtls_ecp_point_free(&Q);

    void* res = be_pushbuffer(vm, out_len);
    memcpy(res, out, out_len);
    be_return(vm);
}

/* add(point1:bytes, point2:bytes) -> bytes(65) - add two points */
static int crypto_ec_p256_add(bvm* vm) {
    crypto_ec_p256_t* ec = (crypto_ec_p256_t*)be_tocomptr(vm, 1);
    size_t p1_len, p2_len;
    const unsigned char* p1_buf =
        (const unsigned char*)be_tobytes(vm, 2, &p1_len);
    const unsigned char* p2_buf =
        (const unsigned char*)be_tobytes(vm, 3, &p2_len);

    mbedtls_ecp_point P1, P2, R;
    mbedtls_ecp_point_init(&P1);
    mbedtls_ecp_point_init(&P2);
    mbedtls_ecp_point_init(&R);

    mbedtls_ecp_point_read_binary(&ec->grp, &P1, p1_buf, p1_len);
    mbedtls_ecp_point_read_binary(&ec->grp, &P2, p2_buf, p2_len);

    mbedtls_mpi one;
    mbedtls_mpi_init(&one);
    mbedtls_mpi_lset(&one, 1);

    mbedtls_ecp_muladd(&ec->grp, &R, &one, &P1, &one, &P2);

    unsigned char out[65];
    size_t out_len;
    mbedtls_ecp_point_write_binary(&ec->grp, &R, MBEDTLS_ECP_PF_UNCOMPRESSED,
                                   &out_len, out, sizeof(out));

    mbedtls_ecp_point_free(&P1);
    mbedtls_ecp_point_free(&P2);
    mbedtls_ecp_point_free(&R);
    mbedtls_mpi_free(&one);

    void* res = be_pushbuffer(vm, out_len);
    memcpy(res, out, out_len);
    be_return(vm);
}

/* @const_object_info_begin
class be_class_crypto_EC_P256 (scope: global, name: EC_P256) {
    _p, var
    init, func(crypto_ec_p256_init)
    deinit, func(crypto_ec_p256_deinit)
    mul, func(crypto_ec_p256_mul)
    mul_point, func(crypto_ec_p256_mul_point)
    add, func(crypto_ec_p256_add)
}

module crypto (scope: global, name: crypto) {
    random, func(crypto_random)
    SHA256, func(crypto_sha256)
    HMAC_SHA256, func(crypto_hmac_sha256)
    HKDF_SHA256, func(crypto_hkdf_sha256)
    PBKDF2_HMAC_SHA256, func(crypto_pbkdf2_hmac_sha256)
    AES_CCM_encrypt, func(crypto_aes_ccm_encrypt)
    AES_CCM_decrypt, func(crypto_aes_ccm_decrypt)
    EC_P256, class(be_class_crypto_EC_P256)
}
@const_object_info_end */

#include "be_fixed_be_class_crypto_EC_P256.h"
#include "be_fixed_crypto.h"
