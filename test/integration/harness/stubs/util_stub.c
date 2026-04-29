#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "controller.h"
#include "util.h"

void debug_hexdump(char dir, const uint8_t* data, unsigned len) {
    (void)data;
    (void)len;
}

void reboot(void) {
    printf("harness: reboot() called — exiting\n");
    fflush(stdout);

    /* Direct semihosting exit to avoid hardfaults in FreeRTOS task context. */
    register int reg0 __asm__("r0") = 0x18;    /* SYS_EXIT */
    register int reg1 __asm__("r1") = 0x20026; /* ADP_Stopped_ApplicationExit */
    __asm__ volatile("bkpt 0xab" : : "r"(reg0), "r"(reg1) : "memory");
    for (;;);
}
