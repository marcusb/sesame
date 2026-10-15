#ifndef MCUBOOT_INFO_H_
#define MCUBOOT_INFO_H_

#include <stdbool.h>
#include <stdint.h>

#define MCUBOOT_INFO_MAGIC 0x4D435542 /* 'MCUB' */
#define MCUBOOT_INFO_ADDR 0x2001FC00

struct mcuboot_info {
    uint32_t magic;       /* MCUBOOT_INFO_MAGIC */
    char version_str[32]; /* e.g. "v2.4.0" */
    uint32_t slot;        /* 0 or 1 */
    uint32_t gen_level;   /* Boot2 partition table gen_level */
    uint32_t confirmed;   /* 1 if confirmed, 0 if test/unconfirmed */
};

/* Trailer stored in the last sector of each mcuboot slot */
#define MCUBOOT_TRAILER_MAGIC 0x4D435554 /* 'MCUT' */
#define MCUBOOT_SLOT_SIZE 0x20000        /* 128 KB */
#define MCUBOOT_SLOT_0_ADDR 0x10000
#define MCUBOOT_SLOT_1_ADDR 0x30000
#define MCUBOOT_TRAILER_OFFSET 0x1F000 /* Last 4KB sector of the 128KB slot */

struct mcuboot_slot_trailer {
    uint32_t magic;      /* MCUBOOT_TRAILER_MAGIC */
    uint32_t boot_count; /* 0 = not booted, >= 1 booted */
    uint32_t confirmed;  /* 1 = confirmed, 0 = unconfirmed test */
    uint32_t reserved;
};

/* Boot2 Partition Table definitions */
#define BOOT2_PARTITION_TABLE_MAGIC 0x54504D57 /* 'WMPT' */
#define BOOT2_PARTITION_TABLE_VERSION 1
#define BOOT2_PART1_ADDR 0x4000
#define BOOT2_PART2_ADDR 0x5000
#define BOOT2_PART_MAX_ENTRIES 10

#define BOOT2_COMP_BOOT2 0
#define BOOT2_COMP_FW 1
#define BOOT2_COMP_WLAN_FW 2
#define BOOT2_COMP_FTFS 3
#define BOOT2_COMP_PSM 4
#define BOOT2_COMP_USER_APP 5

struct boot2_partition_table {
    uint32_t magic;
    uint16_t version;
    uint16_t partition_entries_no;
    uint32_t gen_level;
    uint32_t crc;
};

struct boot2_partition_entry {
    uint8_t type;
    uint8_t device;
    char name[8];
    uint8_t _pad[2];
    uint32_t start;
    uint32_t size;
    uint32_t gen_level;
};

#ifdef __cplusplus
extern "C" {
#endif

extern struct mcuboot_info g_mcuboot_info;
const struct mcuboot_info* mcuboot_get_info(void);

#ifdef __cplusplus
}
#endif

#endif /* MCUBOOT_INFO_H_ */
