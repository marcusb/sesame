#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/devicetree.h>

#include "partition.h"

#define MCUBOOT_INFO_MAGIC 0x4D435542 /* 'MCUB' */

#if DT_NODE_EXISTS(DT_NODELABEL(retained_sram))
#define MCUBOOT_INFO_ADDR DT_REG_ADDR(DT_NODELABEL(retained_sram))
#else
#define MCUBOOT_INFO_ADDR 0x2001FC00
#endif

struct mcuboot_info {
    uint32_t magic;       /* MCUBOOT_INFO_MAGIC */
    char version_str[32]; /* e.g. "v2.4.0" */
    uint32_t slot;        /* 0 or 1 */
    uint32_t gen_level;   /* Boot2 partition table gen_level */
    uint32_t confirmed;   /* 1 if confirmed, 0 if test/unconfirmed */
};

#if DT_NODE_EXISTS(DT_NODELABEL(mcuboot0_partition))
#define MCUBOOT_SLOT_0_ADDR DT_REG_ADDR(DT_NODELABEL(mcuboot0_partition))
#define MCUBOOT_SLOT_SIZE DT_REG_SIZE(DT_NODELABEL(mcuboot0_partition))
#else
#define MCUBOOT_SLOT_0_ADDR 0x10000
#define MCUBOOT_SLOT_SIZE 0x20000 /* 128 KB */
#endif

#if DT_NODE_EXISTS(DT_NODELABEL(mcuboot1_partition))
#define MCUBOOT_SLOT_1_ADDR DT_REG_ADDR(DT_NODELABEL(mcuboot1_partition))
#else
#define MCUBOOT_SLOT_1_ADDR 0x30000
#endif

#ifdef __cplusplus
extern "C" {
#endif

extern struct mcuboot_info g_mcuboot_info;
const struct mcuboot_info* mcuboot_get_info(void);

#ifdef __cplusplus
}
#endif
