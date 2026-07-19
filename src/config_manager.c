#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(config, LOG_LEVEL_DBG);

#include <zephyr/device.h>
#include <zephyr/storage/flash_map.h>

#include "app_config.pb.h"
#include "config_manager.h"
#include "mflash_drv.h"
#include "pb_decode.h"
#include "pb_encode.h"

#define NVS_PARTITION psm_partition
#define NVS_PARTITION_OFFSET PARTITION_OFFSET(NVS_PARTITION)
#define NVS_PARTITION_SIZE PARTITION_SIZE(NVS_PARTITION)

static uint8_t buf[1024] __attribute__((aligned(4)));

AppConfig app_config = AppConfig_init_zero;

// Magic word to check if config is present and valid
#define CONFIG_MAGIC 0x43464731  // "CFG1"

typedef struct {
    uint32_t magic;
    uint32_t length;
    uint32_t checksum;
} config_header_t;

static uint32_t calculate_checksum(const uint8_t* data, size_t len) {
    uint32_t sum = 0;
    for (size_t i = 0; i < len; i++) sum += data[i];
    return sum;
}

int load_config(void) {
    uint32_t config_addr = MFLASH_BASE_ADDRESS + NVS_PARTITION_OFFSET;
    config_header_t* header = (config_header_t*)config_addr;

    if (header->magic != CONFIG_MAGIC || header->length > sizeof(buf)) {
        LOG_WRN("No valid config found");
        return -1;
    }

    const uint8_t* data =
        (const uint8_t*)(config_addr + sizeof(config_header_t));
    if (calculate_checksum(data, header->length) != header->checksum) {
        LOG_ERR("Config checksum mismatch");
        return -1;
    }

    pb_istream_t stream = pb_istream_from_buffer(data, header->length);
    bool status = pb_decode(&stream, AppConfig_fields, &app_config);
    if (!status) {
        LOG_WRN("decode conf object failed: %s", PB_GET_ERROR(&stream));
        return -1;
    }
    return 0;
}

int save_config(void) {
    pb_ostream_t stream = pb_ostream_from_buffer(
        buf + sizeof(config_header_t), sizeof(buf) - sizeof(config_header_t));
    bool status = pb_encode(&stream, AppConfig_fields, &app_config);
    size_t len = stream.bytes_written;
    if (!status) {
        LOG_WRN("encode conf object failed: %s", PB_GET_ERROR(&stream));
        return -1;
    }

    config_header_t* header = (config_header_t*)buf;
    header->magic = CONFIG_MAGIC;
    header->length = len;
    header->checksum = calculate_checksum(buf + sizeof(config_header_t), len);

    size_t total_len = sizeof(config_header_t) + len;
    // Align to 4 bytes for writing
    total_len = (total_len + 3) & ~3;

    // Erase sectors
    mflash_drv_erase(NVS_PARTITION_OFFSET, MFLASH_SECTOR_SIZE);

    // Write data
    int ret = mflash_drv_write(NVS_PARTITION_OFFSET, (uint32_t*)buf, total_len);
    if (ret != 0) {
        LOG_ERR("mflash write config failed %d", ret);
        return -1;
    }
    return 0;
}
