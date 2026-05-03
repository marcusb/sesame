#pragma once

#include <stddef.h>

#define MBEDTLS_NO_PLATFORM_ENTROPY
#define MBEDTLS_ENTROPY_HARDWARE_ALT

#define MBEDTLS_AES_C
#define MBEDTLS_CCM_C
#define MBEDTLS_CIPHER_C
#define MBEDTLS_CTR_DRBG_C
#define MBEDTLS_ENTROPY_C
#define MBEDTLS_MD_C
#define MBEDTLS_OID_C
#define MBEDTLS_PK_C
#define MBEDTLS_PK_PARSE_C
#define MBEDTLS_SHA256_C

#define MBEDTLS_BIGNUM_C
#define MBEDTLS_ECP_C
#define MBEDTLS_ECP_DP_SECP256R1_ENABLED

#define MBEDTLS_PLATFORM_C
#define MBEDTLS_PLATFORM_MEMORY

/* Matter needs these */
#define MBEDTLS_HKDF_C
#define MBEDTLS_ECDH_C
#define MBEDTLS_ECDSA_C
#define MBEDTLS_ECJPAKE_C
#define MBEDTLS_HMAC_DRBG_C
#define MBEDTLS_PKCS5_C

/* Prereqs */
#define MBEDTLS_ASN1_PARSE_C
#define MBEDTLS_ASN1_WRITE_C
#define MBEDTLS_CIPHER_MODE_CBC
#define MBEDTLS_PK_WRITE_C
#define MBEDTLS_PK_CS_C
#define MBEDTLS_X509_CRT_PARSE_C
#define MBEDTLS_X509_USE_C

/* Declarations for platform memory functions */
void *pvPortCalloc(size_t num, size_t size);
void vPortFree(void *ptr);

#define MBEDTLS_PLATFORM_STD_CALLOC pvPortCalloc
#define MBEDTLS_PLATFORM_STD_FREE vPortFree

#define MBEDTLS_ERROR_C
#define MBEDTLS_VERSION_C

