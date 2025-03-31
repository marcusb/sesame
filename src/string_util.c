#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>

// Application
#include "string_util.h"

ssize_t strtcpy(char *restrict dst, const char *restrict src, size_t dsize) {
    bool trunc;
    size_t dlen, slen;

    if (dsize == 0) {
        errno = ENOBUFS;
        return -1;
    }

    slen = strnlen(src, dsize);
    trunc = (slen == dsize);
    dlen = slen - trunc;

    stpcpy(mempcpy(dst, src, dlen), "");
    if (trunc) errno = E2BIG;
    return trunc ? -1 : slen;
}

char *stpecpy(char *dst, char end[0], const char *restrict src) {
    size_t dlen;

    if (dst == NULL) return NULL;

    dlen = strtcpy(dst, src, end - dst);
    return (dlen == -1) ? NULL : dst + dlen;
}
