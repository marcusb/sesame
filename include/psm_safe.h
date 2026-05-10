#pragma once

#include <stdint.h>

#include "psm.h"

/* psm_set_variable on hardware ultimately calls mflash_drv_write, which on
 * MW320 cannot read its source from the same QSPI flash that is being
 * programmed. A debug-build assert in mflash_drv_write halts on any buffer
 * pointer in the XIP-mapped region (0x1F000000-0x1FFFFFFF); release builds
 * silently fail when the QSPI controller stalls.
 *
 * Both the variable *name* and the *value* are written to flash by PSM, so
 * either being a constant in .rodata (typical for Berry-interned strings or
 * inline literals) is enough to crash the device.
 *
 * This wrapper detects XIP-resident inputs and bounces them through a
 * pvPortMalloc'd RAM buffer first. */
int psm_set_variable_safe(psm_hnd_t handle, const char* variable,
                          const void* value, uint32_t len);
