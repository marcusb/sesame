#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include <assert.h>
#include <stdint.h>

#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configTICK_RATE_HZ                      ((TickType_t)1000)
#define configMINIMAL_STACK_SIZE                ((unsigned short)4096)
#define configTOTAL_HEAP_SIZE                   ((size_t)(2 * 1024 * 1024))
#define configMAX_TASK_NAME_LEN                 20
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TASK_NOTIFICATIONS            1
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               8
#define configUSE_QUEUE_SETS                    0
#define configUSE_TIME_SLICING                  1
#define configUSE_NEWLIB_REENTRANT              0
#define configENABLE_BACKWARD_COMPATIBILITY     1
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 5
#define configSUPPORT_STATIC_ALLOCATION         1
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configCHECK_FOR_STACK_OVERFLOW          0
#define configUSE_MALLOC_FAILED_HOOK            0
#define configUSE_APPLICATION_TASK_TAG          0
#define configRECORD_STACK_HIGH_ADDRESS         1

#define configMAX_PRIORITIES                    7

#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               (configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH                20
#define configTIMER_TASK_STACK_DEPTH            (configMINIMAL_STACK_SIZE * 2)

#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xTaskGetIdleTaskHandle          1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xTimerPendFunctionCall          1
#define INCLUDE_xTaskAbortDelay                 1
#define INCLUDE_xTaskGetHandle                  1
#define INCLUDE_xTaskResumeFromISR              1

#define configASSERT(x) assert(x)

/* Logging support used by src/logging.c */
extern void vLoggingPrintf(const char *pcFormat, ...);
#define configPRINTF(X)         vLoggingPrintf X
extern void vLoggingPrint(const char *pcMessage);
#define configPRINT(X)          vLoggingPrint(X)
#define configPRINT_STRING(X)   fputs((X), stderr)
#define configLOGGING_MAX_MESSAGE_LENGTH        256
#define configLOGGING_INCLUDE_TIME_AND_TASK_NAME 1

/* Utility macros used in src/logging.c */
#define max(a,b)                         \
({                                       \
    __typeof__ (a) _a = (a);             \
    __typeof__ (b) _b = (b);             \
    _a > _b ? _a : _b;                   \
})
#define min(a,b)                         \
({                                       \
    __typeof__ (a) _a = (a);             \
    __typeof__ (b) _b = (b);             \
    _a < _b ? _a : _b;                   \
})

#include <stdio.h>

#endif /* FREERTOS_CONFIG_H */
