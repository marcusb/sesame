#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/ztest.h>

#include "config_manager.h"
#include "controller.h"
#include "pic_uart.h"

LOG_MODULE_REGISTER(pic_test, LOG_LEVEL_INF);

K_MSGQ_DEFINE(ctrl_queue, sizeof(ctrl_msg_t), 16, 4);

/* Helper to wait for a stat counter to increase */
static bool wait_for_stat_increment(uint32_t (*get_count)(const pic_stats_t*),
                                    uint32_t prev_count, uint32_t timeout_ms) {
    uint32_t start = k_uptime_get_32();
    pic_stats_t stats;
    while (k_uptime_get_32() - start < timeout_ms) {
        pic_get_stats(&stats);
        if (get_count(&stats) > prev_count) {
            return true;
        }
        k_msleep(20);
    }
    return false;
}

static uint32_t get_version_count(const pic_stats_t* s) {
    return s->version_count;
}
static uint32_t get_door_ack_count(const pic_stats_t* s) {
    return s->door_ack_count;
}
static uint32_t get_alert_ack_count(const pic_stats_t* s) {
    return s->alert_ack_count;
}
static uint32_t get_audio_ack_count(const pic_stats_t* s) {
    return s->audio_ack_count;
}
static uint32_t get_light_ack_count(const pic_stats_t* s) {
    return s->light_ack_count;
}

static void* suite_setup(void) {
    return NULL;
}

static void suite_teardown(void* fixture) {
    ARG_UNUSED(fixture);
    printk("\nTEST_RESULT:0\n");
}

ZTEST_SUITE(pic_test_suite, NULL, suite_setup, NULL, NULL, suite_teardown);

ZTEST(pic_test_suite, test_01_version_query) {
    uint8_t major = 0, minor = 0, patch = 0;
    char suffix = 0;

    /* Wait up to 3s for the initial boot status / version packet, or send a
     * status request */
    pic_stats_t stats;
    pic_get_stats(&stats);
    if (stats.version_count == 0) {
        pic_send_status_req();
    }

    bool received = wait_for_stat_increment(get_version_count, 0, 5000);
    zassert_true(received, "Timed out waiting for PIC version response (0x95)");

    bool ok = pic_get_version(&major, &minor, &patch, &suffix);
    zassert_true(ok, "pic_get_version failed after version packet received");
    LOG_INF("Received PIC Firmware Version: %u.%u.%u+%u", major, minor, patch,
            (uint8_t)suffix);
}

ZTEST(pic_test_suite, test_02_light_on) {
    pic_stats_t stats;
    pic_get_stats(&stats);
    uint32_t prev_acks = stats.light_ack_count;

    int ret = pic_set_light(true);
    zassert_equal(ret, 0, "Failed to enqueue light ON command");

    bool acked = wait_for_stat_increment(get_light_ack_count, prev_acks, 2000);
    zassert_true(acked, "Timed out waiting for PIC light ON ACK (0x93)");
    zassert_true(pic_get_light(),
                 "pic_get_light() returned false after light ON");

    /* Verify ctrl_queue received light state update */
    ctrl_msg_t msg;
    bool found = false;
    while (k_msgq_get(&ctrl_queue, &msg, K_NO_WAIT) == 0) {
        if (msg.type == CTRL_MSG_LIGHT_STATE_UPDATE && msg.msg.light_state.on) {
            found = true;
            break;
        }
    }
    zassert_true(found, "ctrl_queue did not receive light ON state update");
}

ZTEST(pic_test_suite, test_03_light_off) {
    pic_stats_t stats;
    pic_get_stats(&stats);
    uint32_t prev_acks = stats.light_ack_count;

    int ret = pic_set_light(false);
    zassert_equal(ret, 0, "Failed to enqueue light OFF command");

    bool acked = wait_for_stat_increment(get_light_ack_count, prev_acks, 2000);
    zassert_true(acked, "Timed out waiting for PIC light OFF ACK (0x93)");
    zassert_false(pic_get_light(),
                  "pic_get_light() returned true after light OFF");

    /* Verify ctrl_queue received light state update */
    ctrl_msg_t msg;
    bool found = false;
    while (k_msgq_get(&ctrl_queue, &msg, K_NO_WAIT) == 0) {
        if (msg.type == CTRL_MSG_LIGHT_STATE_UPDATE &&
            !msg.msg.light_state.on) {
            found = true;
            break;
        }
    }
    zassert_true(found, "ctrl_queue did not receive light OFF state update");
}

ZTEST(pic_test_suite, test_04_door_open_cmd) {
    pic_stats_t stats;
    pic_get_stats(&stats);
    uint32_t prev_acks = stats.door_ack_count;

    int ret = pic_send_door_raw(1);
    zassert_equal(ret, 0, "Failed to enqueue door open command");

    bool acked = wait_for_stat_increment(get_door_ack_count, prev_acks, 2000);
    zassert_true(acked, "Timed out waiting for PIC door command ACK (0x90)");
}

ZTEST(pic_test_suite, test_05_door_close_cmd) {
    pic_stats_t stats;
    pic_get_stats(&stats);
    uint32_t prev_acks = stats.door_ack_count;

    int ret = pic_send_door_raw(0);
    zassert_equal(ret, 0, "Failed to enqueue door close command");

    bool acked = wait_for_stat_increment(get_door_ack_count, prev_acks, 2000);
    zassert_true(acked, "Timed out waiting for PIC door command ACK (0x90)");
}

ZTEST(pic_test_suite, test_06_audio_cmd) {
    pic_stats_t stats;
    pic_get_stats(&stats);
    uint32_t prev_acks = stats.audio_ack_count;

    int ret = pic_send_audio(2);
    zassert_equal(ret, 0, "Failed to enqueue audio command");

    bool acked = wait_for_stat_increment(get_audio_ack_count, prev_acks, 2000);
    zassert_true(acked, "Timed out waiting for PIC audio command ACK (0x92)");
}

ZTEST(pic_test_suite, test_07_alert_cmd) {
    pic_stats_t stats;
    pic_get_stats(&stats);
    uint32_t prev_acks = stats.alert_ack_count;

    int ret = pic_send_alert(2);
    zassert_equal(ret, 0, "Failed to enqueue alert command");

    bool acked = wait_for_stat_increment(get_alert_ack_count, prev_acks, 2000);
    zassert_true(acked, "Timed out waiting for PIC alert command ACK (0x91)");
}

ZTEST(pic_test_suite, test_08_door_status_req) {
    pic_stats_t stats;
    pic_get_stats(&stats);
    uint32_t prev_versions = stats.version_count;

    int ret = pic_send_status_req();
    zassert_equal(ret, 0, "Failed to enqueue status request");

    bool acked =
        wait_for_stat_increment(get_version_count, prev_versions, 2000);
    zassert_true(acked,
                 "Timed out waiting for PIC status/version response (0x95)");
}
