#ifndef APP_CRYPTO_H
#define APP_CRYPTO_H

#include "mbedtls/ctr_drbg.h"

/* Returns a pointer to the global CTR_DRBG context.
 * The context is automatically initialized on the first call. */
mbedtls_ctr_drbg_context* app_get_global_drbg(void);

#endif /* APP_CRYPTO_H */
