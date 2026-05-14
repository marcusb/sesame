/*
 * <sys/select.h> POSIX shim over FreeRTOS+TCP for CHIP.
 *
 * Include <sys/_timeval.h> before <sys/_select.h> so struct timeval is a
 * complete type when select() is declared.  Without this, GCC 15 treats
 * each struct timeval* parameter as a locally-scoped forward declaration
 * and rejects subsequent compatible declarations as "conflicting types".
 */
#pragma once

#include <sys/_timeval.h>
#include <sys/_select.h>
