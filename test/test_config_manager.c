#include <string.h>

#include "app_config.pb.h"
#include "config_manager.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "string_util.h"
#include "stubs/stub_psm.h"
#include "unity.h"

static void test_load_config_success(void) {
    AppConfig cfg = AppConfig_init_zero;
    cfg.has_network_config = true;
    strtcpy(cfg.network_config.ssid, "test_ssid",
            sizeof(cfg.network_config.ssid));
    strtcpy(cfg.network_config.hostname, "sesame",
            sizeof(cfg.network_config.hostname));

    uint8_t encoded[256];
    pb_ostream_t ostream = pb_ostream_from_buffer(encoded, sizeof(encoded));
    TEST_ASSERT_TRUE(pb_encode(&ostream, AppConfig_fields, &cfg));

    stub_psm_set_data(encoded, ostream.bytes_written);

    memset(&app_config, 0, sizeof(app_config));
    int r = load_config();
    TEST_ASSERT_EQUAL(0, r);
    TEST_ASSERT_TRUE(app_config.has_network_config);
    TEST_ASSERT_EQUAL_STRING("test_ssid", app_config.network_config.ssid);
    TEST_ASSERT_EQUAL_STRING("sesame", app_config.network_config.hostname);
}

static void test_load_config_psm_error(void) {
    stub_psm_set_error(-5);
    memset(&app_config, 0, sizeof(app_config));
    int r = load_config();
    TEST_ASSERT_EQUAL(-5, r);
    stub_psm_set_error(0);
}

static void test_load_config_bad_protobuf(void) {
    uint8_t garbage[] = {0xFF, 0xFE, 0xFD, 0xFC, 0x01, 0x02};
    stub_psm_set_data(garbage, sizeof(garbage));
    memset(&app_config, 0, sizeof(app_config));
    int r = load_config();
    TEST_ASSERT_EQUAL(-1, r);
}

static void test_save_config_roundtrip(void) {
    stub_psm_set_error(0);
    stub_psm_set_data(NULL, 0);

    memset(&app_config, 0, sizeof(app_config));
    app_config.has_mqtt_config = true;
    app_config.mqtt_config.enabled = true;
    strtcpy(app_config.mqtt_config.broker_host, "mqtt.local",
            sizeof(app_config.mqtt_config.broker_host));
    app_config.mqtt_config.broker_port = 1883;

    int r = save_config();
    TEST_ASSERT_EQUAL(0, r);

    const uint8_t* written;
    size_t written_len;
    stub_psm_get_written(&written, &written_len);
    TEST_ASSERT_GREATER_THAN(0, (int)written_len);

    AppConfig decoded = AppConfig_init_zero;
    pb_istream_t istream = pb_istream_from_buffer(written, written_len);
    TEST_ASSERT_TRUE(pb_decode(&istream, AppConfig_fields, &decoded));
    TEST_ASSERT_TRUE(decoded.has_mqtt_config);
    TEST_ASSERT_TRUE(decoded.mqtt_config.enabled);
    TEST_ASSERT_EQUAL_STRING("mqtt.local", decoded.mqtt_config.broker_host);
    TEST_ASSERT_EQUAL(1883, decoded.mqtt_config.broker_port);
}

static void test_save_config_empty(void) {
    stub_psm_set_error(0);
    memset(&app_config, 0, sizeof(app_config));
    int r = save_config();
    TEST_ASSERT_EQUAL(0, r);
}

void run_tests_config_manager(void) {
    UnitySetTestFile(__FILE__);
    RUN_TEST(test_load_config_success);
    RUN_TEST(test_load_config_psm_error);
    RUN_TEST(test_load_config_bad_protobuf);
    RUN_TEST(test_save_config_roundtrip);
    RUN_TEST(test_save_config_empty);
}
