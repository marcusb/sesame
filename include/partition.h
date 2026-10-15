#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

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

#define OTA_TEST_MODE_FLAG (1UL << 31U)

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

/* Standard Marvell soft CRC32 used by Boot2 for partition table and entries */
uint32_t partition_calc_crc32(const uint8_t *data, size_t len);

/* Select active and passive partition table addresses based on generation level and validity */
int partition_get_table_addrs(uint32_t *active_addr, uint32_t *passive_addr);

/* Read a partition table and its entries from flash at table_addr */
int partition_read_table(uint32_t table_addr, struct boot2_partition_table *table,
                         struct boot2_partition_entry *entries, uint32_t max_entries);

/* Read the active partition table and its entries from flash */
int partition_read_active_table(struct boot2_partition_table *table,
                                struct boot2_partition_entry *entries, uint32_t max_entries);

/* Write partition table and entries to flash (at target_addr), calculating CRCs */
int partition_write_table(uint32_t target_addr, struct boot2_partition_table *table,
                          const struct boot2_partition_entry *entries);

/* Find FW partition entries (type BOOT2_COMP_FW). Returns number of entries found (0, 1, or 2). */
int partition_find_fw_entries(const struct boot2_partition_entry *entries, uint32_t num_entries,
                              int fw_indices[2]);

#ifdef __cplusplus
}
#endif
