#include <zephyr/ztest.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>

#include "config_manager.h"

ZTEST_SUITE(config_manager, NULL, NULL, NULL, NULL, NULL);

ZTEST(config_manager, test_save_and_load_network_config) {
    // Modify global config
    strcpy(app_config.network_config.ssid, "test_ssid");
    strcpy(app_config.network_config.password, "test_pass");
    app_config.has_network_config = true;

    // Save
    int ret = save_network_config();
    zassert_equal(ret, 0, "save_network_config failed");

    // Clear memory
    memset(&app_config, 0, sizeof(app_config));

    // Load
    ret = load_config();
    zassert_equal(ret, 0, "load_config failed");

    // Verify
    zassert_true(app_config.has_network_config, "network config not loaded");
    zassert_equal(strcmp(app_config.network_config.ssid, "test_ssid"), 0, "SSID mismatch");
    zassert_equal(strcmp(app_config.network_config.password, "test_pass"), 0, "Password mismatch");
}
