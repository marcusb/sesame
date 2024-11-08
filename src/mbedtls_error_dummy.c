#include "mbedtls/error.h"

static const char DUMMY[] = "<...>";

const char *mbedtls_high_level_strerr(int error_code) {
    return DUMMY;
}

const char *mbedtls_low_level_strerr(int error_code) {
    return DUMMY;
}
