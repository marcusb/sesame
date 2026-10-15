#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "mcuboot_info.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Read Boot2 flags register (0x480C0024) */
uint32_t boot_get_flags(void);

/* Get booted slot (0 or 1) */
uint32_t boot_get_slot(void);

/* Write retained MCUboot info to SRAM */
void boot_info_write(const char *version_str, uint32_t slot, uint32_t gen_level, uint32_t confirmed);

/* Read retained MCUboot info from SRAM. Returns pointer to retained info if valid, or NULL if invalid */
const struct mcuboot_info *boot_info_read(void);

#ifdef __cplusplus
}
#endif
