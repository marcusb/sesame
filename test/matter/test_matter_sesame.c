#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "berry.h"
#include "controller.h"
#include "matter_test_utils.h"
#include "queue.h"
#include "task.h"
#include "unity.h"

extern QueueHandle_t ctrl_queue;

void setUp(void) {
    matter_test_setup();
    ctrl_queue = xQueueCreate(1, sizeof(ctrl_msg_t));
}

void tearDown(void) {
    matter_test_teardown();
    vQueueDelete(ctrl_queue);
}

void test_sesame_door_open(void) {
    be_assert_success("import sesame; sesame.door_cmd(1)");

    ctrl_msg_t msg;
    TEST_ASSERT_EQUAL(pdTRUE, xQueueReceive(ctrl_queue, &msg, 0));
    TEST_ASSERT_EQUAL(CTRL_MSG_DOOR_CONTROL, msg.type);
    TEST_ASSERT_EQUAL(DOOR_CMD_OPEN, msg.msg.door_control.command);
}

void test_sesame_door_close(void) {
    be_assert_success("import sesame; sesame.door_cmd(2)");

    ctrl_msg_t msg;
    TEST_ASSERT_EQUAL(pdTRUE, xQueueReceive(ctrl_queue, &msg, 0));
    TEST_ASSERT_EQUAL(CTRL_MSG_DOOR_CONTROL, msg.type);
    TEST_ASSERT_EQUAL(DOOR_CMD_CLOSE, msg.msg.door_control.command);
}

void test_sesame_door_stop(void) {
    be_assert_success("import sesame; sesame.door_cmd(0)");

    ctrl_msg_t msg;
    TEST_ASSERT_EQUAL(pdTRUE, xQueueReceive(ctrl_queue, &msg, 0));
    TEST_ASSERT_EQUAL(CTRL_MSG_DOOR_CONTROL, msg.type);
    TEST_ASSERT_EQUAL(DOOR_CMD_STOP, msg.msg.door_control.command);
}

/* Runner task to provide FreeRTOS environment */
static void run_tests_task(void* params) {
    (void)params;
    UNITY_BEGIN();
    RUN_TEST(test_sesame_door_open);
    RUN_TEST(test_sesame_door_close);
    RUN_TEST(test_sesame_door_stop);
    int result = UNITY_END();
    printf("\r\nTEST_RESULT:%d\r\n", result);
    fflush(stdout);
    exit(result);
}

extern void test_board_init(void);

int main(void) {
    test_board_init();
    /* Board init simplified for this test */
    if (xTaskCreate(run_tests_task, "tests", 4096, NULL, tskIDLE_PRIORITY + 1,
                    NULL) != pdPASS) {
        return 1;
    }
    vTaskStartScheduler();
    return 0;
}
