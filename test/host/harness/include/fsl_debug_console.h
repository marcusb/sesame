#ifndef FSL_DEBUG_CONSOLE_STUB_H
#define FSL_DEBUG_CONSOLE_STUB_H

#include <stdio.h>

#define PRINTF(...)     fprintf(stderr, __VA_ARGS__)
#define PUTCHAR(c)      fputc((c), stderr)
#define GETCHAR()       fgetc(stdin)

#endif
