#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/kvss/nvs.h>
#include <zephyr/arch/common/semihost.h>
#include <zephyr/sys/printk.h>
#include <string.h>

#include "app_config.pb.h"
#include "pb_decode.h"
#include "pb_encode.h"

#define NVS_ID_NETWORK_CONFIG 1
#define NVS_ID_MQTT_CONFIG 2
#define NVS_ID_LOGGING_CONFIG 3

#ifdef CONFIG_BOARD_NATIVE_SIM
#define NVS_PARTITION storage_partition
#else
#define NVS_PARTITION psm_partition
#endif

#define NVS_PARTITION_DEVICE DEVICE_DT_GET(DT_MTD_FROM_FIXED_PARTITION(DT_NODELABEL(NVS_PARTITION)))
#define NVS_PARTITION_OFFSET DT_REG_ADDR(DT_NODELABEL(NVS_PARTITION))
#define NVS_PARTITION_SIZE DT_REG_SIZE(DT_NODELABEL(NVS_PARTITION))

static struct nvs_fs fs;

static int inject_proto_config(uint16_t id, const pb_msgdesc_t* fields, const void* src, const char* name) {
    static uint8_t buf[256];
    pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
    if (!pb_encode(&stream, fields, src)) {
        printk(">>> encode %s config failed: %s\n", name, PB_GET_ERROR(&stream));
        return -1;
    }
    int ret = nvs_write(&fs, id, buf, stream.bytes_written);
    if (ret < 0) {
        printk(">>> nvs write %s config failed %d\n", name, ret);
        return ret;
    }
    printk(">>> Injected %s config into NVS, size=%zu, rc=%d\n", name, stream.bytes_written, ret);
    return 0;
}

static int inject_test_config(void)
{
    printk(">>> inject_test_config started!\n");
    struct flash_pages_info info;
    const struct device* flash_dev = NVS_PARTITION_DEVICE;
    
    if (!device_is_ready(flash_dev)) {
        printk(">>> Flash device not ready\n");
        return -ENODEV;
    }

    fs.flash_device = flash_dev;
    fs.offset = NVS_PARTITION_OFFSET;
    
    if (flash_get_page_info_by_offs(flash_dev, fs.offset, &info) != 0) {
        printk(">>> Failed to get flash page info\n");
        return -1;
    }
    fs.sector_size = info.size;
    fs.sector_count = NVS_PARTITION_SIZE / info.size;

    if (nvs_mount(&fs) != 0) {
        printk(">>> NVS mount failed in injector\n");
        return -1;
    }

#ifdef CONFIG_SEMIHOST
    printk(">>> Opening test_config.bin via semihosting...\n");
    long fd = semihost_open("test_config.bin", SEMIHOST_OPEN_RB);
    if (fd >= 0) {
        long len = semihost_flen(fd);
        printk(">>> semihost_flen returned %ld\n", len);
        if (len > 0) {
            static uint8_t buf[1024];
            long read_len = semihost_read(fd, buf, len);
            if (len <= sizeof(buf) && read_len == len) {
                static AppConfig app_config; memset(&app_config, 0, sizeof(app_config));
                pb_istream_t stream = pb_istream_from_buffer(buf, len);
                if (pb_decode(&stream, AppConfig_fields, &app_config)) {
                    printk(">>> Successfully decoded AppConfig, ssid=%s\n", app_config.network_config.ssid);
                    
                    if (app_config.has_network_config) {
                        inject_proto_config(NVS_ID_NETWORK_CONFIG, NetworkConfig_fields, &app_config.network_config, "NetworkConfig");
                    }
                    if (app_config.has_mqtt_config) {
                        inject_proto_config(NVS_ID_MQTT_CONFIG, MqttConfig_fields, &app_config.mqtt_config, "MqttConfig");
                    }
                    if (app_config.has_logging_config) {
                        inject_proto_config(NVS_ID_LOGGING_CONFIG, LoggingConfig_fields, &app_config.logging_config, "LoggingConfig");
                    }
                } else {
                    printk(">>> Failed to decode AppConfig: %s\n", PB_GET_ERROR(&stream));
                }
            } else {
                printk(">>> Failed to read test_config.bin, read_len=%ld\n", read_len);
            }
        } else {
            printk(">>> File is empty or flen failed\n");
        }
        semihost_close(fd);
    } else {
        printk(">>> Failed to open test_config.bin, fd=%ld\n", fd);
    }
#else
    printk(">>> CONFIG_SEMIHOST is not defined!\n");
#endif

    return 0;
}

SYS_INIT(inject_test_config, APPLICATION, 80);
