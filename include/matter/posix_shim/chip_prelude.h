/*
 * Forced prelude for every CHIP translation unit (via -include).
 *
 * Sesame's FreeRTOSConfig.h defines `max(a,b)` and `min(a,b)` as macros,
 * which collide with `std::numeric_limits<T>::max()` and similar uses in
 * CHIP. We load FreeRTOSConfig.h here (the real one, via include guard
 * cooperation, so later FreeRTOS.h reincludes are no-ops) and immediately
 * undefine the offending macros so CHIP C++ compiles cleanly. Sesame's
 * own C code still gets max/min via direct FreeRTOSConfig.h includes —
 * unaffected.
 */
#pragma once

#include <FreeRTOSConfig.h>

#ifdef max
#undef max
#endif
#ifdef min
#undef min
#endif
