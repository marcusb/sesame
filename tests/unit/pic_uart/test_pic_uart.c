#include <string.h>
#include <zephyr/ztest.h>

#include "controller.h"
#include "idcm_msg.h"
#include "pic_uart.h"

K_MSGQ_DEFINE(ctrl_queue, sizeof(ctrl_msg_t), 8, 4);

#define MAX_RECORDED_MSGS 16
static dcm_msg_t recorded_msgs[MAX_RECORDED_MSGS];
static size_t recorded_msg_count = 0;

static void test_tx_callback(const dcm_msg_t* msg) {
    if (recorded_msg_count < MAX_RECORDED_MSGS) {
        memcpy(&recorded_msgs[recorded_msg_count++], msg, sizeof(dcm_msg_t));
    }
}

static void reset_test_env(void) {
    pic_uart_test_init();
    pic_set_test_tx_cb(test_tx_callback);
    recorded_msg_count = 0;
    memset(recorded_msgs, 0, sizeof(recorded_msgs));
    k_msgq_purge(&ctrl_queue);
}

static void before_each(void* fixture) {
    ARG_UNUSED(fixture);
    reset_test_env();
}

static void after_each(void* fixture) {
    ARG_UNUSED(fixture);
    k_work_cancel_delayable(pic_get_close_work());
}

ZTEST_SUITE(pic_uart, NULL, NULL, before_each, after_each, NULL);

static bool has_door_cmd(uint8_t val) {
    for (size_t i = 0; i < recorded_msg_count; i++) {
        if (recorded_msgs[i].type == DCM_MSG_DOOR_CMD &&
            recorded_msgs[i].payload.door_cmd.val == val) {
            return true;
        }
    }
    return false;
}

static bool has_alert_cmds(void) {
    bool has_audio = false;
    bool has_alert = false;
    for (size_t i = 0; i < recorded_msg_count; i++) {
        if (recorded_msgs[i].type == DCM_MSG_AUDIO_CMD &&
            recorded_msgs[i].payload.audio_cmd.val == 2) {
            has_audio = true;
        }
        if (recorded_msgs[i].type == DCM_MSG_ALERT_CMD &&
            recorded_msgs[i].payload.alert_cmd.val == 1) {
            has_alert = true;
        }
    }
    return has_audio && has_alert;
}

static size_t count_door_cmds(void) {
    size_t count = 0;
    for (size_t i = 0; i < recorded_msg_count; i++) {
        if (recorded_msgs[i].type == DCM_MSG_DOOR_CMD) {
            count++;
        }
    }
    return count;
}

/* ========================================================================= */
/* Close Command Tests                                                       */
/* ========================================================================= */

ZTEST(pic_uart, test_close_from_open_stopped_schedules_alert) {
    pic_door_state_update(DCM_DOOR_STATE_OPEN, DCM_DOOR_DIR_STOPPED, 32888);

    pic_process_cmd(PIC_CMD_CLOSE);

    zassert_true(pic_is_close_scheduled(), "Close should be scheduled");
    zassert_true(k_work_delayable_is_pending(pic_get_close_work()),
                 "Close delayable work should be pending");
    zassert_true(has_alert_cmds(), "Alert commands should have been sent");
    zassert_equal(count_door_cmds(), 0,
                  "No door command should be sent during alert period");
}

ZTEST(pic_uart, test_close_exec_triggers_door_close) {
    pic_door_state_update(DCM_DOOR_STATE_OPEN, DCM_DOOR_DIR_STOPPED, 32888);
    pic_process_cmd(PIC_CMD_CLOSE);
    zassert_true(pic_is_close_scheduled(), "Close should be scheduled");

    pic_process_cmd(PIC_CMD_CLOSE_EXEC);

    zassert_false(pic_is_close_scheduled(),
                  "Close should no longer be scheduled");
    zassert_true(has_door_cmd(0),
                 "Door CLOSE command (0) should have been sent");
}

ZTEST(pic_uart, test_close_when_already_closed_ignored) {
    pic_door_state_update(DCM_DOOR_STATE_CLOSED, DCM_DOOR_DIR_STOPPED, 32768);

    pic_process_cmd(PIC_CMD_CLOSE);

    zassert_false(pic_is_close_scheduled(), "Close should not be scheduled");
    zassert_equal(recorded_msg_count, 0, "No messages should have been sent");
}

ZTEST(pic_uart, test_close_when_moving_down_ignored) {
    pic_door_state_update(DCM_DOOR_STATE_OPEN, DCM_DOOR_DIR_DOWN, 32850);

    pic_process_cmd(PIC_CMD_CLOSE);

    zassert_false(pic_is_close_scheduled(), "Close should not be scheduled");
    zassert_equal(recorded_msg_count, 0, "No messages should have been sent");
}

ZTEST(pic_uart, test_close_when_moving_up_stops_door) {
    pic_door_state_update(DCM_DOOR_STATE_OPEN, DCM_DOOR_DIR_UP, 32800);

    pic_process_cmd(PIC_CMD_CLOSE);

    zassert_false(pic_is_close_scheduled(),
                  "Close should not be scheduled when moving up");
    zassert_true(has_door_cmd(0),
                 "Door command 0 should have been sent immediately to stop");
}

/* ========================================================================= */
/* Open Command Tests                                                        */
/* ========================================================================= */

ZTEST(pic_uart, test_open_from_closed_sends_open) {
    pic_door_state_update(DCM_DOOR_STATE_CLOSED, DCM_DOOR_DIR_STOPPED, 32768);

    pic_process_cmd(PIC_CMD_OPEN);

    zassert_true(has_door_cmd(1),
                 "Door OPEN command (1) should have been sent");
}

ZTEST(pic_uart, test_open_when_already_open_ignored) {
    dcm_msg_t msg = {
        .type = DCM_MSG_DOOR_STATUS_UPDATE,
        .payload.door_status =
            {
                .state = DCM_DOOR_STATE_OPEN,
                .direction = DCM_DOOR_DIR_STOPPED,
                .pos = 32888,
                .down_limit = 32768,
                .up_limit = 32888,
            },
    };
    pic_handle_msg(&msg);
    zassert_equal(pic_get_pos(), 100, "Door should be at 100%%");

    recorded_msg_count = 0;
    pic_process_cmd(PIC_CMD_OPEN);

    zassert_equal(recorded_msg_count, 0, "No messages should have been sent");
}

ZTEST(pic_uart, test_open_when_moving_up_ignored) {
    pic_door_state_update(DCM_DOOR_STATE_OPEN, DCM_DOOR_DIR_UP, 32800);

    pic_process_cmd(PIC_CMD_OPEN);

    zassert_equal(recorded_msg_count, 0, "No messages should have been sent");
}

ZTEST(pic_uart, test_open_when_moving_down_sends_open) {
    pic_door_state_update(DCM_DOOR_STATE_OPEN, DCM_DOOR_DIR_DOWN, 32850);

    pic_process_cmd(PIC_CMD_OPEN);

    zassert_true(has_door_cmd(1),
                 "Door OPEN command (1) should have been sent");
}

ZTEST(pic_uart, test_open_when_stopped_partway_sends_open) {
    dcm_msg_t msg = {
        .type = DCM_MSG_DOOR_STATUS_UPDATE,
        .payload.door_status =
            {
                .state = DCM_DOOR_STATE_OPEN,
                .direction = DCM_DOOR_DIR_STOPPED,
                .pos = 32828,
                .down_limit = 32768,
                .up_limit = 32888,
            },
    };
    pic_handle_msg(&msg);
    zassert_equal(pic_get_pos(), 50, "Door should be at 50%%");

    recorded_msg_count = 0;
    pic_process_cmd(PIC_CMD_OPEN);

    zassert_true(has_door_cmd(1),
                 "Door OPEN command (1) should have been sent");
}

/* ========================================================================= */
/* Stop Command Tests                                                        */
/* ========================================================================= */

ZTEST(pic_uart, test_stop_when_moving_up_sends_stop) {
    pic_door_state_update(DCM_DOOR_STATE_OPEN, DCM_DOOR_DIR_UP, 32800);

    pic_process_cmd(PIC_CMD_STOP);

    zassert_true(has_door_cmd(1),
                 "Door command 1 should be sent to stop moving door");
}

ZTEST(pic_uart, test_stop_when_moving_down_sends_stop) {
    pic_door_state_update(DCM_DOOR_STATE_OPEN, DCM_DOOR_DIR_DOWN, 32850);

    pic_process_cmd(PIC_CMD_STOP);

    zassert_true(has_door_cmd(1),
                 "Door command 1 should be sent to stop moving door");
}

ZTEST(pic_uart, test_stop_when_already_stopped_ignored) {
    pic_door_state_update(DCM_DOOR_STATE_OPEN, DCM_DOOR_DIR_STOPPED, 32888);

    pic_process_cmd(PIC_CMD_STOP);

    zassert_equal(recorded_msg_count, 0, "No messages should have been sent");
}

/* ========================================================================= */
/* Alert and Close Cancellation Tests                                        */
/* ========================================================================= */

ZTEST(pic_uart, test_cancel_close_on_open_cmd) {
    pic_door_state_update(DCM_DOOR_STATE_OPEN, DCM_DOOR_DIR_STOPPED, 32888);
    pic_process_cmd(PIC_CMD_CLOSE);
    zassert_true(pic_is_close_scheduled(), "Close should be scheduled");

    pic_process_cmd(PIC_CMD_OPEN);

    zassert_false(pic_is_close_scheduled(),
                  "Close should be cancelled on OPEN command");
    zassert_false(k_work_delayable_is_pending(pic_get_close_work()),
                  "Work should be cancelled");

    /* Even if CLOSE_EXEC arrives later, it must be ignored */
    recorded_msg_count = 0;
    pic_process_cmd(PIC_CMD_CLOSE_EXEC);
    zassert_equal(count_door_cmds(), 0, "Stale CLOSE_EXEC should be ignored");
}

ZTEST(pic_uart, test_cancel_close_on_stop_cmd) {
    pic_door_state_update(DCM_DOOR_STATE_OPEN, DCM_DOOR_DIR_STOPPED, 32888);
    pic_process_cmd(PIC_CMD_CLOSE);
    zassert_true(pic_is_close_scheduled(), "Close should be scheduled");

    pic_process_cmd(PIC_CMD_STOP);

    zassert_false(pic_is_close_scheduled(),
                  "Close should be cancelled on STOP command");
    zassert_false(k_work_delayable_is_pending(pic_get_close_work()),
                  "Work should be cancelled");
}

ZTEST(pic_uart, test_cancel_close_on_direction_change) {
    pic_door_state_update(DCM_DOOR_STATE_OPEN, DCM_DOOR_DIR_STOPPED, 32888);
    pic_process_cmd(PIC_CMD_CLOSE);
    zassert_true(pic_is_close_scheduled(), "Close should be scheduled");

    /* PIC reports motor started moving down */
    pic_door_state_update(DCM_DOOR_STATE_OPEN, DCM_DOOR_DIR_DOWN, 32888);

    zassert_false(pic_is_close_scheduled(),
                  "Close should be cancelled on direction change");
    zassert_false(k_work_delayable_is_pending(pic_get_close_work()),
                  "Work should be cancelled");
}

ZTEST(pic_uart, test_cancel_close_on_position_change) {
    pic_door_state_update(DCM_DOOR_STATE_OPEN, DCM_DOOR_DIR_STOPPED, 32888);
    pic_process_cmd(PIC_CMD_CLOSE);
    zassert_true(pic_is_close_scheduled(), "Close should be scheduled");

    /* PIC reports position change */
    pic_door_state_update(DCM_DOOR_STATE_OPEN, DCM_DOOR_DIR_STOPPED, 32850);

    zassert_false(pic_is_close_scheduled(),
                  "Close should be cancelled on position change");
    zassert_false(k_work_delayable_is_pending(pic_get_close_work()),
                  "Work should be cancelled");
}

ZTEST(pic_uart, test_cancel_close_on_state_closed) {
    pic_door_state_update(DCM_DOOR_STATE_OPEN, DCM_DOOR_DIR_STOPPED, 32888);
    pic_process_cmd(PIC_CMD_CLOSE);
    zassert_true(pic_is_close_scheduled(), "Close should be scheduled");

    /* PIC reports state became closed */
    pic_door_state_update(DCM_DOOR_STATE_CLOSED, DCM_DOOR_DIR_STOPPED, 32888);

    zassert_false(pic_is_close_scheduled(),
                  "Close should be cancelled when state is CLOSED");
    zassert_false(k_work_delayable_is_pending(pic_get_close_work()),
                  "Work should be cancelled");
}

ZTEST(pic_uart, test_scheduled_close_retained_when_stationary) {
    pic_door_state_update(DCM_DOOR_STATE_OPEN, DCM_DOOR_DIR_STOPPED, 32888);
    pic_process_cmd(PIC_CMD_CLOSE);
    zassert_true(pic_is_close_scheduled(), "Close should be scheduled");

    /* PIC sends regular status response while door remains stationary */
    pic_door_state_update(DCM_DOOR_STATE_OPEN, DCM_DOOR_DIR_STOPPED, 32888);

    zassert_true(pic_is_close_scheduled(),
                 "Close should remain scheduled when door is stationary");
    zassert_true(k_work_delayable_is_pending(pic_get_close_work()),
                 "Work should remain pending");
}

/* ========================================================================= */
/* Position Calculation and Queue Publishing                                 */
/* ========================================================================= */

ZTEST(pic_uart, test_position_calculation_and_queue_publish) {
    dcm_msg_t msg = {
        .type = DCM_MSG_DOOR_STATUS_UPDATE,
        .payload.door_status =
            {
                .state = DCM_DOOR_STATE_CLOSED,
                .direction = DCM_DOOR_DIR_STOPPED,
                .pos = 32768,
                .down_limit = 32768,
                .up_limit = 32888,
            },
    };

    pic_handle_msg(&msg);
    zassert_equal(pic_get_down_limit(), 32768, "Down limit mismatch");
    zassert_equal(pic_get_up_limit(), 32888, "Up limit mismatch");
    zassert_equal(pic_get_pos(), 0, "Position should be 0%% at down limit");

    ctrl_msg_t cmsg;
    int ret = k_msgq_get(&ctrl_queue, &cmsg, K_NO_WAIT);
    zassert_equal(ret, 0, "Failed to read from ctrl_queue");
    zassert_equal(cmsg.type, CTRL_MSG_DOOR_STATE_UPDATE, "Wrong msg type");
    zassert_equal(cmsg.msg.door_state.state, DCM_DOOR_STATE_CLOSED,
                  "Wrong state");
    zassert_equal(cmsg.msg.door_state.pos, 0, "Wrong pos in ctrl msg");

    /* Test 100% position */
    msg.payload.door_status.state = DCM_DOOR_STATE_OPEN;
    msg.payload.door_status.pos = 32888;
    pic_handle_msg(&msg);
    zassert_equal(pic_get_pos(), 100, "Position should be 100%% at up limit");

    ret = k_msgq_get(&ctrl_queue, &cmsg, K_NO_WAIT);
    zassert_equal(ret, 0, "Failed to read 100%% state from ctrl_queue");
    zassert_equal(cmsg.msg.door_state.pos, 100, "Wrong pos for 100%%");

    /* Test 50% position */
    msg.payload.door_status.pos = 32828;
    pic_handle_msg(&msg);
    zassert_equal(pic_get_pos(), 50, "Position should be 50%% midway");
}

/* ========================================================================= */
/* End-to-End Delayed Work Execution in Zephyr Virtual Time                   */
/* ========================================================================= */

ZTEST(pic_uart, test_delayed_work_timer_execution) {
    pic_door_state_update(DCM_DOOR_STATE_OPEN, DCM_DOOR_DIR_STOPPED, 32888);

    pic_process_cmd(PIC_CMD_CLOSE);
    zassert_true(pic_is_close_scheduled(), "Close should be scheduled");

    /* Drain any prior commands from pic_queue */
    k_msgq_purge(&pic_queue);

    /* Advance time by 6100ms so close_work fires */
    k_sleep(K_MSEC(6100));

    pic_cmd_t cmd = PIC_CMD_UNKNOWN;
    int ret = k_msgq_get(&pic_queue, &cmd, K_NO_WAIT);
    zassert_equal(ret, 0,
                  "Expected PIC_CMD_CLOSE_EXEC on pic_queue after timer");
    zassert_equal(cmd, PIC_CMD_CLOSE_EXEC, "Expected PIC_CMD_CLOSE_EXEC");

    /* Now process the command just as pic_uart_task would */
    pic_process_cmd(cmd);

    zassert_false(pic_is_close_scheduled(),
                  "Close should no longer be scheduled");
    zassert_true(has_door_cmd(0),
                 "Door CLOSE command (0) should have been transmitted");
}
