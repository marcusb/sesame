#include <stdint.h>
#include <string.h>

/* FreeRTOS */
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

/* mw320 board */
#include "board.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"

/* Application */
#include "logging.h"
#include "psm-v2.h"
#include "string_util.h"

/* Unity */
#include "unity.h"

/* Capture backend shared by test_util.c and test_logging.c */
#include "test_capture.h"

/* Global symbols referenced by modules under test */
QueueHandle_t ctrl_queue;
psm_hnd_t psm_hnd = NULL;

/* ---- Unity output ---- */

void unity_putchar(char c) {
    PRINTF("%c", c);
}

/* ---- capture backend ---- */

char    capture_msgs[CAPTURE_MAX_MSGS][CAPTURE_BUF_LEN];
uint8_t capture_levels[CAPTURE_MAX_MSGS];
int     capture_count = 0;

void capture_backend(const log_msg_t *log) {
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

/* ---- FreeRTOS application hooks (main.c not linked in test binary) ---- */

void vApplicationIdleHook(void) {}

void vApplicationMallocFailedHook(void) {
    configASSERT(0);
}

void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName) {
    (void)xTask;
    (void)pcTaskName;
    configASSERT(0);
}

/* ---- Unity setUp / tearDown (one definition for entire binary) ---- */

void setUp(void) {}
void tearDown(void) {}

/* ---- Test suite forward declarations ---- */

void run_test_string_util(void);
void run_test_util(void);
void run_test_logging(void);
void run_test_config_manager(void);
void run_test_pic_uart(void);

/* ---- Test runner task ---- */

static void run_tests_task(void *params) {
    (void)params;

    register_log_backend(capture_backend);
    init_logging(512, tskIDLE_PRIORITY, 16);
    vTaskDelay(pdMS_TO_TICKS(50));

    UNITY_BEGIN();
    run_test_string_util();
    run_test_util();
    run_test_logging();
    run_test_config_manager();
    run_test_pic_uart();
    int result = UNITY_END();

    PRINTF("\r\nTEST_RESULT:%d\r\n", result);
    vTaskDelete(NULL);
}

/* ---- Heap setup (mirrors main.c) ---- */

extern unsigned __HeapBase, __HeapLimit, __HeapBase_sram0, __HeapLimit_sram0;

static void setup_heap(void) {
    HeapRegion_t xHeapRegions[] = {
        {(uint8_t *)&__HeapBase_sram0,
         (unsigned)&__HeapLimit_sram0 - (unsigned)&__HeapBase_sram0},
        {(uint8_t *)&__HeapBase,
         (unsigned)&__HeapLimit - (unsigned)&__HeapBase},
        {NULL, 0}};
    vPortDefineHeapRegions(xHeapRegions);
}

/* ---- Entry point ---- */

int main(void) {
    board_init_pins();
    init_boot_clocks();
    init_debug_console();

    setup_heap();

    if (xTaskCreate(run_tests_task, "tests", configMINIMAL_STACK_SIZE + 512,
                    NULL, tskIDLE_PRIORITY + 1, NULL) != pdPASS) {
        PRINTF("test task creation failed\r\n");
    }
    vTaskStartScheduler();
    return 0;
}
