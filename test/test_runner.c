#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* FreeRTOS */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* mw320 board */
#include "board.h"
#include "clock_config.h"
#include "pin_mux.h"

/* Application */
#include "board_support.h"
#include "logging.h"
#include "psm-v2.h"
#include "string_util.h"

/* Unity */
#include "unity.h"

/* Capture backend shared by test_util.c and test_logging.c */
#include "test_capture.h"

/* ---- capture backend ---- */

char capture_msgs[CAPTURE_MAX_MSGS][CAPTURE_BUF_LEN];
uint8_t capture_levels[CAPTURE_MAX_MSGS];
int capture_count = 0;

void capture_backend(const log_msg_t* log) {
    if (!log->msg || capture_count >= CAPTURE_MAX_MSGS) return;
    strtcpy(capture_msgs[capture_count], log->msg, CAPTURE_BUF_LEN);
    capture_levels[capture_count] = log->level;
    capture_count++;
}

void capture_reset(void) {
    memset(capture_msgs, 0, sizeof(capture_msgs));
    memset(capture_levels, 0, sizeof(capture_levels));
    capture_count = 0;
}

void capture_drain_and_reset(void) {
    vTaskDelay(pdMS_TO_TICKS(50));
    capture_reset();
}

/* ---- Unity setUp / tearDown (one definition for entire binary) ---- */

__attribute__((weak)) void setUp(void) {}
__attribute__((weak)) void tearDown(void) {}

/* ---- Test suite hook ---- */

extern void run_tests(void);

/* ---- Test runner task ---- */

static void run_tests_task(void* params) {
    (void)params;

    register_log_backend(capture_backend);
    init_logging(1024, tskIDLE_PRIORITY, 16);

    UNITY_BEGIN();
    run_tests();
    int result = UNITY_END();

    char sentinel[32];
    int len =
        snprintf(sentinel, sizeof(sentinel), "\r\nTEST_RESULT:%d\r\n", result);
    write(1, sentinel, len);
    exit(result);
}

/* ---- Entry point ---- */

int main(void) {
    board_init();

    if (xTaskCreate(run_tests_task, "tests", 16384, NULL, tskIDLE_PRIORITY + 1,
                    NULL) != pdPASS) {
        printf("test task creation failed\r\n");
    }
    vTaskStartScheduler();
    return 0;
}
