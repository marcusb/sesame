#include <stddef.h>
#include <unistd.h>

#include "unity_config.h"

/* ---- Unity output port (Semihosting) ---- */

static char unity_buf[256];
static size_t unity_buf_ptr = 0;

void unity_putchar(char c) {
    unity_buf[unity_buf_ptr++] = c;
    if (c == '\n' || unity_buf_ptr >= sizeof(unity_buf)) {
        write(1, unity_buf, unity_buf_ptr);
        unity_buf_ptr = 0;
    }
}

void unity_flush(void) {
    if (unity_buf_ptr > 0) {
        write(1, unity_buf, unity_buf_ptr);
        unity_buf_ptr = 0;
    }
}
