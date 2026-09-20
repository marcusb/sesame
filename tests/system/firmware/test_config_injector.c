#include <string.h>
#include <zephyr/arch/common/semihost.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/printk.h>

#include "app_config.pb.h"
#include "pb_decode.h"
#include "pb_encode.h"

static int inject_proto_config(const char* key, const pb_msgdesc_t* fields,
                               const void* src, const char* name) {
    static uint8_t buf[256];
    pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
    if (!pb_encode(&stream, fields, src)) {
        printk(">>> encode %s config failed: %s\n", name,
               PB_GET_ERROR(&stream));
        return -1;
    }
    int ret = settings_save_one(key, buf, stream.bytes_written);
    if (ret != 0) {
        printk(">>> settings_save_one %s failed: %d\n", key, ret);
        return ret;
    }
    printk(">>> Injected %s config into settings (%s), size=%zu\n", name, key,
           stream.bytes_written);
    return 0;
}

static int inject_test_config(void) {
    printk(">>> inject_test_config started!\n");
    int rc = settings_subsys_init();
    if (rc != 0) {
        printk(">>> settings_subsys_init failed in injector: %d\n", rc);
        return rc;
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
                static AppConfig app_config;
                memset(&app_config, 0, sizeof(app_config));
                pb_istream_t stream = pb_istream_from_buffer(buf, len);
                if (pb_decode(&stream, AppConfig_fields, &app_config)) {
                    printk(">>> Successfully decoded AppConfig, ssid=%s\n",
                           app_config.network_config.ssid);

                    if (app_config.has_network_config) {
                        inject_proto_config(
                            "sesame/network", NetworkConfig_fields,
                            &app_config.network_config, "NetworkConfig");
                    }
                    if (app_config.has_mqtt_config) {
                        inject_proto_config("sesame/mqtt", MqttConfig_fields,
                                            &app_config.mqtt_config,
                                            "MqttConfig");
                    }
                    if (app_config.has_logging_config) {
                        inject_proto_config(
                            "sesame/logging", LoggingConfig_fields,
                            &app_config.logging_config, "LoggingConfig");
                    }
                } else {
                    printk(">>> Failed to decode AppConfig: %s\n",
                           PB_GET_ERROR(&stream));
                }
            } else {
                printk(">>> Failed to read test_config.bin, read_len=%ld\n",
                       read_len);
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
