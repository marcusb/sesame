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

#ifdef CONFIG_BOARD_NATIVE_SIM
#define NVS_PARTITION storage_partition
#else
#define NVS_PARTITION psm_partition
#endif

#define NVS_PARTITION_DEVICE \
    DEVICE_DT_GET(DT_MTD_FROM_FIXED_PARTITION(DT_NODELABEL(NVS_PARTITION)))
#define NVS_PARTITION_OFFSET DT_REG_ADDR(DT_NODELABEL(NVS_PARTITION))
#define NVS_PARTITION_SIZE DT_REG_SIZE(DT_NODELABEL(NVS_PARTITION))

#define NVS_ID_NETWORK_CONFIG 1
#define NVS_ID_MQTT_CONFIG 2
#define NVS_ID_LOGGING_CONFIG 3

static struct nvs_fs fs;
static bool nvs_initialized = false;
static union {
    uint32_t words[256];
    uint8_t bytes[1024];
} buf_union;
#define buf (buf_union.bytes)

AppConfig app_config = AppConfig_init_zero;

static int init_nvs(void) {
    if (nvs_initialized) {
        return 0;
    }

    struct flash_pages_info info;
    fs.flash_device = NVS_PARTITION_DEVICE;
    if (!device_is_ready(fs.flash_device)) {
        LOG_ERR("Flash device not ready");
        return -ENODEV;
    }

    fs.offset = NVS_PARTITION_OFFSET;

    if (flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info)) {
        LOG_ERR("Failed to get flash page info");
        return -EINVAL;
    }

    fs.sector_size = info.size;
    fs.sector_count = NVS_PARTITION_SIZE / info.size;

    int rc = nvs_mount(&fs);
    if (rc) {
        LOG_WRN("Flash mount failed (%d), erasing NVS partition...", rc);
        rc = flash_erase(fs.flash_device, fs.offset, NVS_PARTITION_SIZE);
        if (rc) {
            LOG_ERR("Failed to erase NVS partition: %d", rc);
            return rc;
        }
        rc = nvs_mount(&fs);
        if (rc) {
            LOG_ERR("Flash mount failed again after erase: %d", rc);
            return rc;
        }
    }
    nvs_initialized = true;
    return 0;
}

static void load_proto_config(uint16_t id, const pb_msgdesc_t* fields,
                              void* dest, bool* has_config, const char* name) {
    int ret = nvs_read(&fs, id, buf, sizeof(buf));
    if (ret >= 0) {
        LOG_INF("Loaded %s config, size %d bytes", name, ret);
        pb_istream_t stream = pb_istream_from_buffer(buf, ret);
        if (pb_decode(&stream, fields, dest)) {
            *has_config = true;
        } else {
            LOG_WRN("decode %s config failed: %s", name, PB_GET_ERROR(&stream));
        }
    } else {
        LOG_INF("Failed to load %s config: %d", name, ret);
    }
}

int load_config(void) {
    if (init_nvs() != 0) {
        return -1;
    }

    load_proto_config(NVS_ID_NETWORK_CONFIG, NetworkConfig_fields,
                      &app_config.network_config,
                      &app_config.has_network_config, "network");

    load_proto_config(NVS_ID_MQTT_CONFIG, MqttConfig_fields,
                      &app_config.mqtt_config, &app_config.has_mqtt_config,
                      "mqtt");

    load_proto_config(NVS_ID_LOGGING_CONFIG, LoggingConfig_fields,
                      &app_config.logging_config,
                      &app_config.has_logging_config, "logging");

    return 0;
}

static int save_proto_config(uint16_t id, const pb_msgdesc_t* fields,
                             const void* src, const char* name) {
    LOG_INF("Entering save_proto_config for %s", name);
    if (init_nvs() != 0) {
        LOG_ERR("init_nvs failed");
        return -1;
    }
    pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
    if (!pb_encode(&stream, fields, src)) {
        LOG_WRN("encode %s config failed: %s", name, PB_GET_ERROR(&stream));
        return -1;
    }
    LOG_INF("About to nvs_write %d bytes to id %d", stream.bytes_written, id);
    int ret = nvs_write(&fs, id, buf, stream.bytes_written);
    LOG_INF("nvs_write returned %d", ret);
    if (ret < 0) {
        LOG_ERR("nvs write %s config failed %d", name, ret);
        return ret;
    }
    LOG_INF("Saved %s config, size %d bytes", name, stream.bytes_written);
    return 0;
}

int save_network_config(void) {
    return save_proto_config(NVS_ID_NETWORK_CONFIG, NetworkConfig_fields,
                             &app_config.network_config, "network");
}

int save_mqtt_config(void) {
    return save_proto_config(NVS_ID_MQTT_CONFIG, MqttConfig_fields,
                             &app_config.mqtt_config, "mqtt");
}

int save_logging_config(void) {
    return save_proto_config(NVS_ID_LOGGING_CONFIG, LoggingConfig_fields,
                             &app_config.logging_config, "logging");
}
