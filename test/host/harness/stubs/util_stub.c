#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "controller.h"
#include "util.h"

void debug_hexdump(char dir, const uint8_t* data, unsigned len) {
    (void)data;
    (void)len;
    fprintf(stderr, "hexdump %c (%u bytes)\n", dir, len);
}

void reboot(void) {
    fprintf(stderr, "harness: reboot() called — exiting\n");
    fflush(stderr);
    fflush(stdout);
    _exit(0);
}
