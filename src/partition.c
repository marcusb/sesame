#include "partition.h"

#include <errno.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kernel.h>

uint32_t partition_calc_crc32(const uint8_t* data, size_t len) {
    static const uint32_t table[16] = {
        0x00000000U, 0x1db71064U, 0x3b6e20c8U, 0x26d930acU,
        0x76dc4190U, 0x6b6b51f4U, 0x4db26158U, 0x5005713cU,
        0xedb88320U, 0xf00f9344U, 0xd6d6a3e8U, 0xcb61b38cU,
        0x9b64c2b0U, 0x86d3d2d4U, 0xa00ae278U, 0xbdbdf21cU,
    };
    uint32_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        uint8_t byte = data[i];
        crc = (crc >> 4) ^ table[(crc ^ byte) & 0x0f];
        crc = (crc >> 4) ^ table[(crc ^ ((uint32_t)byte >> 4)) & 0x0f];
    }
    return crc;
}

static const struct device* get_flash_dev(void) {
    const struct device* dev =
        DEVICE_DT_GET(DT_CHOSEN(zephyr_flash_controller));
    if (!device_is_ready(dev)) {
        return NULL;
    }
    return dev;
}

int partition_read_table(uint32_t table_addr,
                         struct boot2_partition_table* table,
                         struct boot2_partition_entry* entries,
                         uint32_t max_entries) {
    const struct device* dev = get_flash_dev();
    if (!dev) {
        return -ENODEV;
    }

    if (!table) {
        return -EINVAL;
    }

    int rc = flash_read(dev, table_addr, table, sizeof(*table));
    if (rc != 0) {
        return -EIO;
    }

    if (table->magic != BOOT2_PARTITION_TABLE_MAGIC) {
        return -EBADMSG;
    }

    uint32_t calc_crc =
        partition_calc_crc32((const uint8_t*)table, sizeof(*table) - 4);
    if (calc_crc != table->crc) {
        return -EBADMSG;
    }

    if (entries && max_entries > 0) {
        uint32_t num = table->partition_entries_no;
        if (num > max_entries) {
            num = max_entries;
        }
        rc = flash_read(dev, table_addr + sizeof(*table), entries,
                        num * sizeof(struct boot2_partition_entry));
        if (rc != 0) {
            return -EIO;
        }
    }
    return 0;
}

int partition_get_table_addrs(uint32_t* active_addr, uint32_t* passive_addr) {
    const struct device* dev = get_flash_dev();
    if (!dev) {
        return -ENODEV;
    }

    struct boot2_partition_table pt[2];
    bool pt0_valid =
        (partition_read_table(BOOT2_PART1_ADDR, &pt[0], NULL, 0) == 0);
    bool pt1_valid =
        (partition_read_table(BOOT2_PART2_ADDR, &pt[1], NULL, 0) == 0);

    uint32_t active = BOOT2_PART1_ADDR;
    uint32_t passive = BOOT2_PART2_ADDR;

    if ((pt0_valid && pt1_valid && pt[1].gen_level > pt[0].gen_level) ||
        (!pt0_valid && pt1_valid)) {
        active = BOOT2_PART2_ADDR;
        passive = BOOT2_PART1_ADDR;
    }

    if (active_addr) {
        *active_addr = active;
    }
    if (passive_addr) {
        *passive_addr = passive;
    }
    return 0;
}

int partition_read_active_table(struct boot2_partition_table* table,
                                struct boot2_partition_entry* entries,
                                uint32_t max_entries) {
    uint32_t active_addr = 0;
    partition_get_table_addrs(&active_addr, NULL);
    return partition_read_table(active_addr, table, entries, max_entries);
}

int partition_write_table(uint32_t target_addr,
                          struct boot2_partition_table* table,
                          const struct boot2_partition_entry* entries) {
    const struct device* dev = get_flash_dev();
    if (!dev) {
        return -ENODEV;
    }

    if (!table || !entries) {
        return -EINVAL;
    }

    table->magic = BOOT2_PARTITION_TABLE_MAGIC;
    table->version = BOOT2_PARTITION_TABLE_VERSION;
    table->crc =
        partition_calc_crc32((const uint8_t*)table, sizeof(*table) - 4);

    uint32_t e_size =
        table->partition_entries_no * sizeof(struct boot2_partition_entry);
    uint32_t e_crc = partition_calc_crc32((const uint8_t*)entries, e_size);

    int rc = flash_erase(dev, target_addr, 4096);
    if (rc != 0) {
        return -EIO;
    }

    rc = flash_write(dev, target_addr, table, sizeof(*table));
    if (rc != 0) {
        return -EIO;
    }

    rc = flash_write(dev, target_addr + sizeof(*table), entries, e_size);
    if (rc != 0) {
        return -EIO;
    }

    rc = flash_write(dev, target_addr + sizeof(*table) + e_size, &e_crc,
                     sizeof(e_crc));
    return (rc == 0) ? 0 : -EIO;
}

int partition_find_fw_entries(const struct boot2_partition_entry* entries,
                              uint32_t num_entries, int fw_indices[2]) {
    if (fw_indices) {
        fw_indices[0] = -1;
        fw_indices[1] = -1;
    }
    if (!entries) {
        return 0;
    }

    int fw_count = 0;
    for (uint32_t i = 0; i < num_entries; i++) {
        if (entries[i].type == BOOT2_COMP_FW) {
            if (fw_count < 2) {
                if (fw_indices) {
                    fw_indices[fw_count] = (int)i;
                }
                fw_count++;
            }
        }
    }
    return fw_count;
}
