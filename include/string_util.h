#pragma once

#include <stddef.h>
#include <stdio.h>
#include <sys/types.h>

ssize_t strtcpy(char *dst, const char *src, size_t dsize);
char *stpecpy(char *dst, char end[0], const char *src);
