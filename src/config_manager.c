#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(config, LOG_LEVEL_DBG);

#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/kvss/nvs.h>
#include <zephyr/storage/flash_map.h>

#include "app_config.pb.h"
#include "config_manager.h"
#include "pb_decode.h"
#include "pb_encode.h"

#define NVS_PARTITION psm_partition
#define NVS_PARTITION_DEVICE PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET PARTITION_OFFSET(NVS_PARTITION)
#define NVS_PARTITION_SIZE PARTITION_SIZE(NVS_PARTITION)
#define NVS_ID_APP_CONFIG 1

static struct nvs_fs fs;
static bool nvs_initialized = false;
static uint8_t buf[1024];

AppConfig app_config = AppConfig_init_zero;

static int init_nvs(void) {
    if (nvs_initialized) return 0;

    struct flash_pages_info info;
    fs.flash_device = NVS_PARTITION_DEVICE;
    if (!device_is_ready(fs.flash_device)) {
        LOG_ERR("Flash device not ready");
        return -ENODEV;
    }
    fs.offset = NVS_PARTITION_OFFSET;

    // NVS needs sector configuration, get it from flash pages info
    if (flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info)) {
        LOG_ERR("Failed to get flash page info");
        return -EINVAL;
    }

    fs.sector_size = info.size;
    fs.sector_count = NVS_PARTITION_SIZE / info.size;

    int rc = nvs_mount(&fs);
    if (rc) {
        LOG_ERR("Flash mount failed: %d", rc);
        return rc;
    }
    nvs_initialized = true;
    return 0;
}

int load_config(void) {
    if (init_nvs() != 0) return -1;

    int ret = nvs_read(&fs, NVS_ID_APP_CONFIG, buf, sizeof(buf));
    if (ret <= 0) {
        LOG_DBG("nvs read config failed or not found %d", ret);
        return -1;
    }

    pb_istream_t stream = pb_istream_from_buffer(buf, ret);
    bool status = pb_decode(&stream, AppConfig_fields, &app_config);
    if (!status) {
        LOG_WRN("decode conf object failed: %s", PB_GET_ERROR(&stream));
        return -1;
    }
    return 0;
}

int save_config(void) {
    if (init_nvs() != 0) return -1;

    pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
    bool status = pb_encode(&stream, AppConfig_fields, &app_config);
    size_t len = stream.bytes_written;
    if (!status) {
        LOG_WRN("encode conf object failed: %s", PB_GET_ERROR(&stream));
        return -1;
    }

    int ret = nvs_write(&fs, NVS_ID_APP_CONFIG, buf, len);
    if (ret < 0) {
        LOG_ERR("nvs write config failed %d", ret);
        return ret;
    }
    return 0;
}
