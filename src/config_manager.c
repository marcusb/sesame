#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(config, LOG_LEVEL_DBG);

#include <errno.h>
#include <string.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/util.h>

#include "app_config.pb.h"
#include "config_manager.h"
#include "pb_decode.h"
#include "pb_encode.h"

AppConfig app_config = AppConfig_init_zero;

struct proto_config_entry {
    const char* key;
    const pb_msgdesc_t* fields;
    void* dest;
    bool* has_config;
};

static const struct proto_config_entry config_entries[] = {
    {"network", NetworkConfig_fields, &app_config.network_config,
     &app_config.has_network_config},
    {"mqtt", MqttConfig_fields, &app_config.mqtt_config,
     &app_config.has_mqtt_config},
    {"logging", LoggingConfig_fields, &app_config.logging_config,
     &app_config.has_logging_config},
};

static int sesame_settings_set(const char* name, size_t len,
                               settings_read_cb read_cb, void* cb_arg) {
    const char* next;
    uint8_t buf[512];

    for (size_t i = 0; i < ARRAY_SIZE(config_entries); i++) {
        const struct proto_config_entry* entry = &config_entries[i];
        if (settings_name_steq(name, entry->key, &next) && !next) {
            if (len > sizeof(buf)) {
                LOG_ERR("%s config too large (%zu bytes)", entry->key, len);
                return -EINVAL;
            }
            ssize_t rc = read_cb(cb_arg, buf, len);
            if (rc <= 0) {
                LOG_ERR("read_cb failed for %s: %zd", entry->key, rc);
                return -EIO;
            }
            pb_istream_t stream = pb_istream_from_buffer(buf, (size_t)rc);
            if (pb_decode(&stream, entry->fields, entry->dest)) {
                *entry->has_config = true;
                LOG_INF("Loaded %s config (%zd bytes)", entry->key, rc);
                return 0;
            } else {
                LOG_WRN("decode %s config failed: %s", entry->key,
                        PB_GET_ERROR(&stream));
                return -EINVAL;
            }
        }
    }

    return -ENOENT;
}

SETTINGS_STATIC_HANDLER_DEFINE(sesame, "sesame", NULL, sesame_settings_set,
                               NULL, NULL);

int load_config(void) {
    int rc = settings_subsys_init();
    if (rc != 0) {
        LOG_ERR("settings_subsys_init failed: %d", rc);
        return rc;
    }

    rc = settings_load_subtree("sesame");
    if (rc != 0) {
        LOG_WRN("settings_load_subtree failed: %d", rc);
        return rc;
    }

    return 0;
}

static int save_proto_config(const char* name, const pb_msgdesc_t* fields,
                             const void* src, const char* log_name) {
    uint8_t buf[512];
    int rc = settings_subsys_init();
    if (rc != 0) {
        LOG_ERR("settings_subsys_init failed: %d", rc);
        return rc;
    }

    pb_ostream_t stream = pb_ostream_from_buffer(buf, sizeof(buf));
    if (!pb_encode(&stream, fields, src)) {
        LOG_WRN("encode %s config failed: %s", log_name, PB_GET_ERROR(&stream));
        return -EINVAL;
    }

    rc = settings_save_one(name, buf, stream.bytes_written);
    if (rc != 0) {
        LOG_ERR("settings_save_one %s failed: %d", name, rc);
        return rc;
    }

    LOG_INF("Saved %s config (%zu bytes)", log_name, stream.bytes_written);
    return 0;
}

int save_network_config(void) {
    return save_proto_config("sesame/network", NetworkConfig_fields,
                             &app_config.network_config, "network");
}

int save_mqtt_config(void) {
    return save_proto_config("sesame/mqtt", MqttConfig_fields,
                             &app_config.mqtt_config, "mqtt");
}

int save_logging_config(void) {
    return save_proto_config("sesame/logging", LoggingConfig_fields,
                             &app_config.logging_config, "logging");
}
