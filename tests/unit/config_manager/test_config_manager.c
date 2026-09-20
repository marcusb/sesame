#include <string.h>
#include <zephyr/ztest.h>

#include "config_manager.h"

ZTEST_SUITE(config_manager, NULL, NULL, NULL, NULL, NULL);

ZTEST(config_manager, test_save_and_load_network_config) {
    memset(&app_config, 0, sizeof(app_config));

    strcpy(app_config.network_config.ssid, "test_ssid");
    strcpy(app_config.network_config.password, "test_pass");
    app_config.network_config.security = WifiSecurity_WIFI_SECURITY_WPA;
    app_config.has_network_config = true;

    int ret = save_network_config();
    zassert_equal(ret, 0, "save_network_config failed: %d", ret);

    memset(&app_config, 0, sizeof(app_config));

    ret = load_config();
    zassert_equal(ret, 0, "load_config failed: %d", ret);

    zassert_true(app_config.has_network_config, "network config not loaded");
    zassert_equal(strcmp(app_config.network_config.ssid, "test_ssid"), 0,
                  "SSID mismatch");
    zassert_equal(strcmp(app_config.network_config.password, "test_pass"), 0,
                  "Password mismatch");
    zassert_equal(app_config.network_config.security,
                  WifiSecurity_WIFI_SECURITY_WPA, "Security mismatch");
}

ZTEST(config_manager, test_save_and_load_mqtt_config) {
    memset(&app_config, 0, sizeof(app_config));

    app_config.mqtt_config.enabled = true;
    strcpy(app_config.mqtt_config.broker_host, "mqtt.example.com");
    app_config.mqtt_config.broker_port = 1883;
    strcpy(app_config.mqtt_config.prefix, "sesame");
    app_config.has_mqtt_config = true;

    int ret = save_mqtt_config();
    zassert_equal(ret, 0, "save_mqtt_config failed: %d", ret);

    memset(&app_config, 0, sizeof(app_config));

    ret = load_config();
    zassert_equal(ret, 0, "load_config failed: %d", ret);

    zassert_true(app_config.has_mqtt_config, "mqtt config not loaded");
    zassert_true(app_config.mqtt_config.enabled, "mqtt enabled mismatch");
    zassert_equal(
        strcmp(app_config.mqtt_config.broker_host, "mqtt.example.com"), 0,
        "Broker host mismatch");
    zassert_equal(app_config.mqtt_config.broker_port, 1883,
                  "Broker port mismatch");
    zassert_equal(strcmp(app_config.mqtt_config.prefix, "sesame"), 0,
                  "Prefix mismatch");
}

ZTEST(config_manager, test_save_and_load_logging_config) {
    memset(&app_config, 0, sizeof(app_config));

    app_config.logging_config.has_syslog_config = true;
    app_config.logging_config.syslog_config.enabled = true;
    strcpy(app_config.logging_config.syslog_config.syslog_host,
           "syslog.example.com");
    app_config.logging_config.syslog_config.syslog_port = 514;
    app_config.has_logging_config = true;

    int ret = save_logging_config();
    zassert_equal(ret, 0, "save_logging_config failed: %d", ret);

    memset(&app_config, 0, sizeof(app_config));

    ret = load_config();
    zassert_equal(ret, 0, "load_config failed: %d", ret);

    zassert_true(app_config.has_logging_config, "logging config not loaded");
    zassert_true(app_config.logging_config.has_syslog_config,
                 "syslog config missing");
    zassert_true(app_config.logging_config.syslog_config.enabled,
                 "syslog enabled mismatch");
    zassert_equal(strcmp(app_config.logging_config.syslog_config.syslog_host,
                         "syslog.example.com"),
                  0, "Syslog host mismatch");
    zassert_equal(app_config.logging_config.syslog_config.syslog_port, 514,
                  "Syslog port mismatch");
}
