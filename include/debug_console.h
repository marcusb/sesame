#ifndef SESAME_DEBUG_CONSOLE_H
#define SESAME_DEBUG_CONSOLE_H

#ifdef QEMU
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#define SCANF(...)  scanf(__VA_ARGS__)
#define PUTCHAR(c)  putchar(c)
#define GETCHAR()   getchar()
#else
#include "fsl_debug_console.h"
#endif

#endif // SESAME_DEBUG_CONSOLE_H
